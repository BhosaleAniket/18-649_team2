/*
 * control.c - safety state, drive motors, and steering servo
 *
 * One high-priority thread owns every actuator. It wakes either when the
 * RX ISR delivers a new command (for the 2 ms response) or every 20 ms
 * (to measure speed, integrate the PI loop, and detect link loss).
 *
 * Each wake, in order:
 *   1. faults: power-up, link loss, bad frames, self-test button
 *   2. if any fault  -> fail-safe: motor braked, steering centered
 *   3. else brake pedal -> dynamic brake (checked before throttle, always wins)
 *   4. else throttle -> PI speed control
 *   5. steering servo
 *
 * L298N drive (PWM on EN, IN pins select the state):
 *   EN(PA10)  IN1(PC0)  IN2(PA11)   State
 *   PWM(d)    1         0           Forward, speed d (off-time coasts)
 *   100%      0         0           Dynamic brake, PWM disabled
 *   0%        x         x           Coast
 * The command link has no reverse field, so the car drives forward only.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "control.h"
#include "link.h"
#include "link_proto.h"
#include "config.h"

/* ------------------------------------------------------------------ */
/* Hardware                                                           */
/* ------------------------------------------------------------------ */

static const struct device *qdec1_dev = DEVICE_DT_GET(DT_NODELABEL(qdec));
static const struct device *qdec2_dev = DEVICE_DT_GET(DT_NODELABEL(qdec2));
static const struct device *motor_pwm = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const struct device *servo_pwm = DEVICE_DT_GET(DT_NODELABEL(pwm10));

static const struct gpio_dt_spec in1 = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_in1), gpios);
static const struct gpio_dt_spec in2 = GPIO_DT_SPEC_GET(DT_NODELABEL(motor_in2), gpios);
static const struct gpio_dt_spec tp_pwm_set = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_pwm_set), gpios);

#define MOTOR_PWM_CHANNEL   3       /* TIM1_CH3, PA10 */
#define MOTOR_PERIOD_CYCLES 4200    /* 84 MHz / 4200 = 20 kHz */
#define SERVO_PWM_CHANNEL   1       /* TIM10_CH1, PB8 */

/* ------------------------------------------------------------------ */
/* Tuning and calibration                                             */
/* ------------------------------------------------------------------ */

/* Encoders */
#define COUNTS_PER_WHEEL_REV  660
#define ENC1_SIGN             (1)
#define ENC2_SIGN             (-1)
#define RPM_FILTER_ALPHA      0.3f

/* Throttle and brake (pedal units 0..255) */
#define MAX_TARGET_RPM     200.0f   /* full throttle; set to ~85% of measured top speed */
#define THROTTLE_DEADBAND  5        /* pedal noise at rest */
#define BRAKE_THRESHOLD    25       /* above this, brake */

/* PI */
#define MAX_DUTY     0.95f          /* near full; keep stall tests brief (L298N ~2 A/ch) */
#define KP           0.010f
#define KI           0.050f
#define INTEG_LIMIT  0.8f
#define DT_S         (CONTROL_PERIOD_MS / 1000.0f)

/* Servo, in servo-shaft degrees.
 * ESTIMATE, not calibrated: observed +/-18 deg servo -> ~5 deg wheel,
 * so ~3.6 servo deg per wheel deg. Wheel stops are ~20 deg; targeting
 * ~17 deg wheel for margin -> +/-60 deg servo around center.
 * If the servo buzzes at either end, move that end 5 deg toward center. */
#define SERVO_LEFT_DEG     30.0f
#define SERVO_CENTER_DEG   90.0f
#define SERVO_RIGHT_DEG    150.0f
#define SERVO_LO_DEG (SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_LEFT_DEG : SERVO_RIGHT_DEG)
#define SERVO_HI_DEG (SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_RIGHT_DEG : SERVO_LEFT_DEG)

/* ------------------------------------------------------------------ */
/* Published state (written here, read by other threads)              */
/* ------------------------------------------------------------------ */

static atomic_t faults_pub  = ATOMIC_INIT(ST_FAULT_POWERUP);
static atomic_t rpm_x10_pub;
static atomic_t target_x10_pub;
static atomic_t duty_pct_pub;
static atomic_t braking_pub;

uint8_t control_get_faults(void)
{
    return (uint8_t)atomic_get(&faults_pub);
}

uint8_t control_get_state(void)
{
    return control_get_faults() ? ST_STATE_ERROR : ST_STATE_NORMAL;
}

void control_get_telemetry(struct control_telemetry *out)
{
    out->rpm_x10    = (int)atomic_get(&rpm_x10_pub);
    out->target_x10 = (int)atomic_get(&target_x10_pub);
    out->duty_pct   = (int)atomic_get(&duty_pct_pub);
    out->braking    = (int)atomic_get(&braking_pub);
}

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static int32_t count_delta(uint32_t now, uint32_t prev)
{
    /* Low 16 bits are correct for a 16- or 32-bit counter */
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
    return (delta * 60000.0f) / (COUNTS_PER_WHEEL_REV * CONTROL_PERIOD_MS);
}

/* Documented, monotonic: 0..DEADBAND -> 0, then linear to MAX_TARGET_RPM at 255 */
static float throttle_to_target_rpm(uint8_t thr)
{
    if (thr <= THROTTLE_DEADBAND) {
        return 0.0f;
    }
    return ((thr - THROTTLE_DEADBAND) / (255.0f - THROTTLE_DEADBAND)) * MAX_TARGET_RPM;
}

/* Documented, monotonic: piecewise linear through left / center / right.
 * steer = +/-STEER_FULL_SCALE reaches full lock; beyond that it saturates. */
static float steer_to_servo_deg(int16_t steer)
{
    float x = clampf(steer / (float)STEER_FULL_SCALE, -1.0f, 1.0f);

    if (x >= 0.0f) {
        return SERVO_CENTER_DEG + x * (SERVO_RIGHT_DEG - SERVO_CENTER_DEG);
    }
    return SERVO_CENTER_DEG + (-x) * (SERVO_LEFT_DEG - SERVO_CENTER_DEG);
}

/* ------------------------------------------------------------------ */
/* Actuators                                                          */
/* ------------------------------------------------------------------ */

static void set_en_duty(float d)
{
    d = clampf(d, 0.0f, 1.0f);
    pwm_set_cycles(motor_pwm, MOTOR_PWM_CHANNEL, MOTOR_PERIOD_CYCLES,
                   (uint32_t)(d * MOTOR_PERIOD_CYCLES), 0);
    gpio_pin_toggle_dt(&tp_pwm_set);    /* PWM_SET: right after the timer write */
    atomic_set(&duty_pct_pub, (atomic_val_t)(d * 100.0f));
}

static void motor_forward(float duty)
{
    gpio_pin_set_dt(&in1, 1);
    gpio_pin_set_dt(&in2, 0);
    set_en_duty(clampf(duty, 0.0f, MAX_DUTY));
    atomic_set(&braking_pub, 0);
}

static void motor_brake(void)
{
    gpio_pin_set_dt(&in1, 0);           /* DIR_A edge */
    gpio_pin_set_dt(&in2, 0);
    set_en_duty(1.0f);                  /* EN held high, no modulation */
    atomic_set(&braking_pub, 1);
}

static void servo_write_deg(float deg)
{
    deg = clampf(deg, SERVO_LO_DEG, SERVO_HI_DEG);
    float us = 500.0f + deg * (2000.0f / 180.0f);
    pwm_set(servo_pwm, SERVO_PWM_CHANNEL, PWM_USEC(20000),
            PWM_USEC((uint32_t)(us + 0.5f)), 0);
}

/* ------------------------------------------------------------------ */
/* PI controller                                                      */
/* ------------------------------------------------------------------ */

static float integ;

/* Integrates only on the 20 ms tick, so extra wakes from commands give an
 * immediate proportional response without changing the loop's timing. */
static float pi_update(float target, float measured, bool integrate)
{
    if (target <= 0.0f) {
        integ = 0.0f;
        return 0.0f;
    }

    float err = target - measured;
    float u_unsat = KP * err + integ;
    float u = clampf(u_unsat, 0.0f, MAX_DUTY);

    if (integrate) {
        bool windup = (u_unsat > MAX_DUTY && err > 0.0f) ||
                      (u_unsat < 0.0f && err < 0.0f);
        if (!windup) {
            integ = clampf(integ + KI * err * DT_S, -INTEG_LIMIT, INTEG_LIMIT);
        }
    }
    return u;
}

/* ------------------------------------------------------------------ */
/* Fault / error state                                                */
/* ------------------------------------------------------------------ */

static uint8_t update_faults(const struct link_cmd *cmd, int64_t now)
{
    static bool    have_seq;
    static uint8_t last_seq;

    static bool    test_level;
    static int64_t test_change_ms = -100000;
    static int64_t last_press_ms  = -100000;
    static bool    selftest;

    static uint32_t last_bad;
    static int64_t  badcmd_until;

    uint8_t faults = 0;

    /* Power-up: error until the first valid command. Link loss after that. */
    if (!cmd->valid) {
        faults |= ST_FAULT_POWERUP;
    } else if ((now - cmd->rx_ms) > FAIL_SAFE_MS) {
        faults |= ST_FAULT_LINK;
    }

    /* Any rejected frame holds the error state for BADCMD_HOLD_MS */
    uint32_t bad = link_bad_frames();
    if (bad != last_bad) {
        last_bad = bad;
        badcmd_until = now + BADCMD_HOLD_MS;
    }
    if (now < badcmd_until) {
        faults |= ST_FAULT_BADCMD;
    }

    /* Self-test button: level in each new frame, debounced here.
     * Single press enters self-test immediately; a double press leaves it. */
    bool new_frame = cmd->valid && (!have_seq || cmd->seq != last_seq);
    if (new_frame) {
        have_seq = true;
        last_seq = cmd->seq;

        bool level = (cmd->flags & CMD_FLAG_TEST) != 0;
        if (level != test_level && (now - test_change_ms) >= BUTTON_DEBOUNCE_MS) {
            test_level = level;
            test_change_ms = now;
            if (level) {                            /* press */
                if (!selftest) {
                    selftest = true;
                } else if ((now - last_press_ms) <= DOUBLE_PRESS_MS) {
                    selftest = false;
                }
                last_press_ms = now;
            }
        }
    }
    if (selftest) {
        faults |= ST_FAULT_SELFTEST;
    }

    return faults;
}

/* ------------------------------------------------------------------ */
/* Control thread                                                     */
/* ------------------------------------------------------------------ */

K_THREAD_STACK_DEFINE(control_stack, STACK_CONTROL);
static struct k_thread control_thread_data;

static void control_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint32_t prev1 = read_count(qdec1_dev);
    uint32_t prev2 = read_count(qdec2_dev);
    float rpm_filt = 0.0f;

    int64_t next_tick = k_uptime_get() + CONTROL_PERIOD_MS;

    while (1) {
        /* Sleep until the next tick, or wake early on a new command */
        int64_t now = k_uptime_get();
        if (next_tick > now) {
            k_sem_take(&link_cmd_sem, K_MSEC((int32_t)(next_tick - now)));
            now = k_uptime_get();
        }

        bool tick = false;
        if (now >= next_tick) {
            tick = true;
            next_tick += CONTROL_PERIOD_MS;
            if (next_tick <= now) {
                next_tick = now + CONTROL_PERIOD_MS;   /* fell behind: resync */
            }
        }

        /* Measure speed on ticks only (the formula assumes a fixed period) */
        if (tick) {
            uint32_t n1 = read_count(qdec1_dev);
            uint32_t n2 = read_count(qdec2_dev);
            int32_t d1 = ENC1_SIGN * count_delta(n1, prev1);
            int32_t d2 = ENC2_SIGN * count_delta(n2, prev2);
            prev1 = n1;
            prev2 = n2;

            float rpm_avg = 0.5f * (counts_to_rpm(d1) + counts_to_rpm(d2));
            rpm_filt += RPM_FILTER_ALPHA * (rpm_avg - rpm_filt);
            atomic_set(&rpm_x10_pub, (atomic_val_t)(rpm_filt * 10.0f));
        }

        struct link_cmd cmd;
        link_get_command(&cmd);

        uint8_t faults = update_faults(&cmd, now);
        atomic_set(&faults_pub, faults);

        if (faults != 0) {
            /* Fail-safe: braked, steering centered */
            integ = 0.0f;
            motor_brake();
            servo_write_deg(SERVO_CENTER_DEG);
            atomic_set(&target_x10_pub, 0);
            continue;
        }

        /* Brake first: it wins over throttle under all circumstances */
        if (cmd.brk > BRAKE_THRESHOLD) {
            integ = 0.0f;
            motor_brake();
            atomic_set(&target_x10_pub, 0);
        } else {
            float target = throttle_to_target_rpm(cmd.thr);
            atomic_set(&target_x10_pub, (atomic_val_t)(target * 10.0f));
            motor_forward(pi_update(target, rpm_filt, tick));
        }

        /* Steering after the time-critical drive outputs */
        servo_write_deg(steer_to_servo_deg(cmd.steer));
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

int control_init(void)
{
    if (!device_is_ready(qdec1_dev) || !device_is_ready(qdec2_dev) ||
        !device_is_ready(motor_pwm) || !device_is_ready(servo_pwm) ||
        !gpio_is_ready_dt(&in1) || !gpio_is_ready_dt(&in2) ||
        !gpio_is_ready_dt(&tp_pwm_set)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&in1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&in2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&tp_pwm_set, GPIO_OUTPUT_INACTIVE);

    /* Power-up is the error state: braked, centered */
    motor_brake();
    servo_write_deg(SERVO_CENTER_DEG);

    k_thread_create(&control_thread_data, control_stack,
                    K_THREAD_STACK_SIZEOF(control_stack),
                    control_thread, NULL, NULL, NULL,
                    PRIO_CONTROL, 0, K_NO_WAIT);
    k_thread_name_set(&control_thread_data, "control");
    return 0;
}