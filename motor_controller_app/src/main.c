/*
 * motor_controller_app - closed-loop wheel speed control (Nucleo-F401RE + L298N)
 *
 * Pin map (STM32 side):
 *   PB4  TIM3_CH1  <- encoder 1 A        PB5  TIM3_CH2  <- encoder 1 B
 *   PA5  TIM2_CH1  <- encoder 2 A        PA1  TIM2_CH2  <- encoder 2 B
 *   PA10 TIM1_CH3  -> L298N ENA + ENB (speed PWM)
 *   PC0  GPIO      -> L298N IN1 + IN3 (direction)
 *   PA11 GPIO      -> L298N IN2 + IN4 (direction)
 *   PC13 GPIO      <- user button (active low), steps the bench test
 *
 * L298N drive scheme ("tutorial style": PWM on EN, IN pins select state):
 *
 *   EN (PA10)   IN1 (PC0)  IN2 (PA11)   State
 *   PWM(d)      1          0            Forward, speed = d  (off-time coasts)
 *   PWM(d)      0          1            Reverse, speed = d  (off-time coasts)
 *   100%        0          0            Dynamic brake (PWM disabled, EN held high)
 *   0%          x          x            Coast (bridge off)
 *
 * Velocity for control = average of the two wheel speeds (per the spec).
 * Brake takes priority over any speed command.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

/* ------------------------------------------------------------------ */
/* Hardware                                                           */
/* ------------------------------------------------------------------ */

static const struct device *qdec1_dev = DEVICE_DT_GET(DT_NODELABEL(qdec));
static const struct device *qdec2_dev = DEVICE_DT_GET(DT_NODELABEL(qdec2));
static const struct device *pwm_dev   = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *gpioc_dev = DEVICE_DT_GET(DT_NODELABEL(gpioc));

#define PWM_CHANNEL        3      /* TIM1_CH3 on PA10 -> ENA + ENB           */
#define PWM_PERIOD_CYCLES  4200   /* 84 MHz / 4200 = 20 kHz (verify on scope) */

#define IN1_PIN     0             /* PC0  -> IN1 + IN3 */
#define IN2_PIN     11            /* PA11 -> IN2 + IN4 */
#define BUTTON_PIN  13            /* PC13, active low  */

/* ------------------------------------------------------------------ */
/* Measurement                                                        */
/* ------------------------------------------------------------------ */

#define SAMPLE_PERIOD_MS      20
#define DT_S                  (SAMPLE_PERIOD_MS / 1000.0f)
#define COUNTS_PER_WHEEL_REV  660     /* measured */
#define ENC1_SIGN             (1)
#define ENC2_SIGN             (-1)    /* motors are mirrored; verify with push test */
#define RPM_FILTER_ALPHA      0.3f    /* 1.0 = no filtering */

/* ------------------------------------------------------------------ */
/* Control                                                            */
/* ------------------------------------------------------------------ */

#define MAX_TARGET_RPM   70.0f   /* full throttle; set below measured 100%-duty speed */
#define MAX_DUTY         0.8f    /* cap to limit current near stall (L298N ~2 A/ch)   */
#define KP               0.010f  /* duty per RPM of error          (starting point)   */
#define KI               0.050f  /* duty per RPM*s of error        (starting point)   */
#define INTEG_LIMIT      0.8f
#define REVERSE_OK_RPM   3.0f    /* must be nearly stopped before reversing */

#define TEST_RPM         40.0f   /* bench test setpoint */

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Signed difference between two counter readings, correct across wraparound.
 * Taking the low 16 bits works whether the timer wraps at 16 or 32 bits,
 * as long as fewer than 32768 counts occur per sample (true here). */
static int32_t count_delta(uint32_t now, uint32_t prev)
{
    return (int16_t)(uint16_t)(now - prev);
}

static uint32_t read_count(const struct device *dev)
{
    struct sensor_value v;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_ENCODER_COUNT, &v);
    return (uint32_t)v.val1;
}

static float counts_to_rpm(int32_t delta)
{
    return (delta * 60000.0f) / (COUNTS_PER_WHEEL_REV * SAMPLE_PERIOD_MS);
}

/* Documented, monotonic throttle mapping for Pi-link integration:
 * throttle 0..255 maps linearly to 0..MAX_TARGET_RPM. */
static inline float throttle_to_target_rpm(uint8_t thr)
{
    return (thr / 255.0f) * MAX_TARGET_RPM;
}

/* ------------------------------------------------------------------ */
/* Motor output - one function per L298N state                        */
/* ------------------------------------------------------------------ */

static void set_en_duty(float d)
{
    d = clampf(d, 0.0f, 1.0f);
    pwm_set_cycles(pwm_dev, PWM_CHANNEL, PWM_PERIOD_CYCLES,
                   (uint32_t)(d * PWM_PERIOD_CYCLES), 0);
}

static void motor_forward(float duty)
{
    gpio_pin_set(gpioc_dev, IN1_PIN, 1);
    gpio_pin_set(gpioa_dev, IN2_PIN, 0);
    set_en_duty(clampf(duty, 0.0f, MAX_DUTY));
}

static void motor_reverse(float duty)
{
    gpio_pin_set(gpioc_dev, IN1_PIN, 0);
    gpio_pin_set(gpioa_dev, IN2_PIN, 1);
    set_en_duty(clampf(duty, 0.0f, MAX_DUTY));
}

static void motor_brake(void)
{
    gpio_pin_set(gpioc_dev, IN1_PIN, 0);
    gpio_pin_set(gpioa_dev, IN2_PIN, 0);
    set_en_duty(1.0f);                  /* EN held high, no modulation */
}

static void motor_coast(void)
{
    set_en_duty(0.0f);                  /* EN held low, bridge off */
}

/* Signed command in [-1, 1] */
static void set_motor(float u)
{
    if (u >= 0.0f) {
        motor_forward(u);
    } else {
        motor_reverse(-u);
    }
}

/* ------------------------------------------------------------------ */
/* PI controller                                                      */
/* ------------------------------------------------------------------ */

struct pi_ctrl {
    float kp;
    float ki;
    float integ;
};

/* Output limited to the target's sign: the car slows by coasting,
 * never by actively driving the opposite way. */
static float pi_step(struct pi_ctrl *c, float target, float measured)
{
    if (target == 0.0f) {
        c->integ = 0.0f;
        return 0.0f;
    }

    float lo = (target > 0.0f) ? 0.0f : -MAX_DUTY;
    float hi = (target > 0.0f) ? MAX_DUTY : 0.0f;

    float err = target - measured;
    float u_unsat = c->kp * err + c->integ;
    float u = clampf(u_unsat, lo, hi);

    /* Anti-windup: don't integrate further into saturation. */
    bool pushing_high = (u_unsat > hi) && (err > 0.0f);
    bool pushing_low  = (u_unsat < lo) && (err < 0.0f);

    if (!pushing_high && !pushing_low) {
        c->integ = clampf(c->integ + c->ki * err * DT_S,
                          -INTEG_LIMIT, INTEG_LIMIT);
    }

    return u;
}

/* True if the wheels are still turning opposite to the requested direction. */
static bool moving_wrong_way(float target, float rpm)
{
    return (target > 0.0f && rpm < -REVERSE_OK_RPM) ||
           (target < 0.0f && rpm >  REVERSE_OK_RPM);
}

/* One control step. Brake is checked first and overrides everything. */
static float control_step(struct pi_ctrl *pi, float target, float rpm, bool brake)
{
    if (brake) {
        pi->integ = 0.0f;
        motor_brake();
        return 0.0f;
    }

    if (moving_wrong_way(target, rpm)) {
        pi->integ = 0.0f;
        motor_brake();                  /* stop before reversing */
        return 0.0f;
    }

    float u = pi_step(pi, target, rpm);
    set_motor(u);
    return u;
}

/* ------------------------------------------------------------------ */
/* Bench test sequence (user button steps through it)                 */
/* ------------------------------------------------------------------ */

enum step_kind { STEP_COAST, STEP_DRIVE, STEP_BRAKE };

struct test_step {
    const char *name;
    enum step_kind kind;
    float target_rpm;   /* signed, used by STEP_DRIVE (negative = reverse) */
    bool timed_stop;    /* print how long the wheels take to stop */
};

/* Each button press advances one step, then wraps back to the start. */
static const struct test_step test_steps[] = {
    { "IDLE (coast)",           STEP_COAST,  0.0f,      false },
    { "FORWARD",                STEP_DRIVE,  TEST_RPM,  false },
    { "BRAKE",                  STEP_BRAKE,  0.0f,      true  },
    { "FORWARD",                STEP_DRIVE,  TEST_RPM,  false },
    { "COAST",                  STEP_COAST,  0.0f,      true  },
    { "REVERSE",                STEP_DRIVE, -TEST_RPM,  false },
    { "FORWARD (from reverse)", STEP_DRIVE,  TEST_RPM,  false },
    { "BRAKE",                  STEP_BRAKE,  0.0f,      true  },
};
#define NUM_STEPS (sizeof(test_steps) / sizeof(test_steps[0]))

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

static bool ready(const struct device *dev, const char *name)
{
    if (!device_is_ready(dev)) {
        printk("ERROR: %s not ready\n", name);
        return false;
    }
    return true;
}

int main(void)
{
    if (!ready(qdec1_dev, "QDEC1") || !ready(qdec2_dev, "QDEC2") ||
        !ready(pwm_dev, "PWM") || !ready(gpioa_dev, "GPIOA") ||
        !ready(gpioc_dev, "GPIOC")) {
        return 0;
    }

    gpio_pin_configure(gpioc_dev, IN1_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioa_dev, IN2_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioc_dev, BUTTON_PIN, GPIO_INPUT);
    motor_coast();                      /* nothing moves at boot */

    struct pi_ctrl pi = { .kp = KP, .ki = KI, .integ = 0.0f };

    uint32_t prev1 = read_count(qdec1_dev);
    uint32_t prev2 = read_count(qdec2_dev);
    float rpm_filt = 0.0f;

    size_t step = 0;
    bool prev_pressed = false;

    bool reversing = false;             /* auto-braking before a direction change */
    int64_t reversal_start_ms = 0;

    bool timing_stop = false;
    int64_t stop_start_ms = 0;
    int zero_samples = 0;

    int print_div = 0;
    int64_t next_sample = k_uptime_get() + SAMPLE_PERIOD_MS;

    printk("motor_controller_app ready, step 0 -> %s\n", test_steps[step].name);

    while (1) {
        /* Fixed-schedule 20 ms loop */
        int64_t now_ms = k_uptime_get();
        if (now_ms < next_sample) {
            k_msleep((int32_t)(next_sample - now_ms));
        }
        next_sample += SAMPLE_PERIOD_MS;

        /* Button: step the test sequence on each press (20 ms sampling debounces) */
        bool pressed = (gpio_pin_get(gpioc_dev, BUTTON_PIN) == 0);
        if (pressed && !prev_pressed) {
            step = (step + 1) % NUM_STEPS;
            pi.integ = 0.0f;            /* fresh start for each step */
            reversing = false;
            timing_stop = false;

            if (test_steps[step].timed_stop) {
                stop_start_ms = k_uptime_get();
                timing_stop = true;
                zero_samples = 0;
            }
            printk("step %d -> %s\n", (int)step, test_steps[step].name);
        }
        prev_pressed = pressed;
        const struct test_step *s = &test_steps[step];

        /* Measure */
        uint32_t now1 = read_count(qdec1_dev);
        uint32_t now2 = read_count(qdec2_dev);
        int32_t d1 = ENC1_SIGN * count_delta(now1, prev1);
        int32_t d2 = ENC2_SIGN * count_delta(now2, prev2);
        prev1 = now1;
        prev2 = now2;

        float rpm1 = counts_to_rpm(d1);
        float rpm2 = counts_to_rpm(d2);
        float rpm_avg = 0.5f * (rpm1 + rpm2);
        rpm_filt += RPM_FILTER_ALPHA * (rpm_avg - rpm_filt);

        /* Control */
        float u = 0.0f;
        switch (s->kind) {
        case STEP_DRIVE: {
            /* Log when control_step auto-brakes before a direction change */
            bool wrong_way = moving_wrong_way(s->target_rpm, rpm_filt);
            if (wrong_way && !reversing) {
                reversing = true;
                reversal_start_ms = k_uptime_get();
                printk("auto-brake: still moving the other way\n");
            } else if (!wrong_way && reversing) {
                reversing = false;
                printk("reversal: braked in %d ms, now driving\n",
                       (int)(k_uptime_get() - reversal_start_ms));
            }
            u = control_step(&pi, s->target_rpm, rpm_filt, false);
            break;
        }
        case STEP_BRAKE:
            u = control_step(&pi, 0.0f, rpm_filt, true);
            break;
        case STEP_COAST:
        default:
            pi.integ = 0.0f;
            motor_coast();
            break;
        }

        /* Stop timing: stopped = no counts on either wheel for 3 samples */
        if (timing_stop) {
            zero_samples = (d1 == 0 && d2 == 0) ? zero_samples + 1 : 0;
            if (zero_samples >= 3) {
                int stop_ms = (int)(k_uptime_get() - stop_start_ms)
                              - 2 * SAMPLE_PERIOD_MS;
                printk("%s: stopped in %d ms\n", s->name, stop_ms);
                timing_stop = false;
            }
        }

        /* Log every 100 ms. RPM x10 and duty in %, as integers (no float printf). */
        if (++print_div >= 5) {
            print_div = 0;
            printk("rpm1=%d rpm2=%d avg=%d duty=%d%% integ=%d\n",
                   (int)(rpm1 * 10), (int)(rpm2 * 10), (int)(rpm_filt * 10),
                   (int)(u * 100), (int)(pi.integ * 1000));
        }
    }

    return 0;
}