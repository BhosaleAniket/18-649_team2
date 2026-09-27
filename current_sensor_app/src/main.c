/*
 * current_sensor_app - lab 3.5 current sensor bench test (Nucleo-F401RE)
 *
 * Reads the three current sensors at 1 kHz, converts to mA, filters, and
 * prints them. The user button steps through the three sanity-check
 * conditions and prints a summary for each, so you get the three numbers
 * per sensor the lab asks for.
 *
 * Pins (see PINMAP.md):
 *   PA0  ADC1_IN0  <- left motor current
 *   PA4  ADC1_IN4  <- right motor current
 *   PB0  ADC1_IN8  <- servo current
 *   PA10 TIM1_CH3  -> L298N ENA + ENB (speed PWM)
 *   PC0  GPIO      -> L298N IN1 + IN3
 *   PA11 GPIO      -> L298N IN2 + IN4
 *   PB8  TIM10_CH1 -> steering servo
 *   PC13 GPIO      <- user button (active low), steps the test
 *
 * Test steps (button advances):
 *   REST   motors coast, servo held at center
 *   FREE   motors driven open loop, wheels off the ground; servo sweeps
 *   STALL  hold the wheels and the steering BEFORE pressing; motors driven,
 *          servo pushed toward full left. Cuts out on its own after
 *          STALL_MAX_MS so nothing is held stalled for long.
 *
 * Serial console (ST-LINK COM port, 115200) keys:
 *   f   cycle the display filter (shows its time constant)
 *   z   re-zero the sensors (only with everything at rest)
 *
 * Fault thresholds (requirements DD-17 / DD-18, report only, no action):
 *   open circuit  < 50 mA for 200 ms while PWM >= 20%
 *   overload      > 2000 mA for 20 ms
 * Unplug a motor during FREE to check the open-circuit threshold.
 *
 * Conversion (linear sensor, see the table below):
 *   V_sensor = V_adc * divider
 *   I        = (V_sensor - V_zero) / sensitivity
 * V_zero is measured at boot with everything off, so the sensitivity is
 * the only number you need from the sensor datasheet.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

/* ------------------------------------------------------------------ */
/* Hardware                                                           */
/* ------------------------------------------------------------------ */

#define USER_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_chans[] = {
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, i_left),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, i_right),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, i_servo),
};
#define NUM_CH  3
BUILD_ASSERT(ARRAY_SIZE(adc_chans) == NUM_CH);

static const struct device *motor_pwm = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const struct device *servo_pwm = DEVICE_DT_GET(DT_NODELABEL(pwm10));
static const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *gpioc_dev = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *console   = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define MOTOR_CHANNEL      3      /* TIM1_CH3 on PA10 */
#define PWM_PERIOD_CYCLES  4200   /* 84 MHz / 4200 = 20 kHz, same as motor app */
#define IN1_PIN            0      /* PC0  */
#define IN2_PIN            11     /* PA11 */
#define BUTTON_PIN         13     /* PC13, active low */

#define SERVO_CHANNEL      1      /* TIM10_CH1 on PB8 */
#define SERVO_PERIOD_US    20000
#define SERVO_LEFT_US      1300   /* keep in sync with servo_app */
#define SERVO_CENTER_US    1500
#define SERVO_RIGHT_US     1700

/* ------------------------------------------------------------------ */
/* Sensors - fill in from the datasheet and your wiring               */
/* ------------------------------------------------------------------ */

#define ADC_VREF_MV   3300.0f     /* VREF+ = VDDA = 3.3 V on the Nucleo */
#define ADC_FULL      4096.0f     /* 12 bit */

struct sensor_cal {
    const char *name;
    float mv_per_a;     /* sensitivity at the sensor output. PLACEHOLDER */
    float divider;      /* sensor V / ADC pin V (1.0 = wired directly)   */
};

/* PLACEHOLDERS: e.g. ACS712-05B = 185 mV/A, -20A = 100 mV/A, -30A = 66 mV/A.
 * A 5 V sensor needs a divider so the ADC pin never sees more than 3.3 V;
 * set divider to (R1 + R2) / R2 in that case. */
static const struct sensor_cal cal[] = {
    { "L", 185.0f, 1.0f },
    { "R", 185.0f, 1.0f },
    { "S", 185.0f, 1.0f },
};

/* ------------------------------------------------------------------ */
/* Timing                                                             */
/* ------------------------------------------------------------------ */

#define SAMPLE_PERIOD_US   1000   /* 1 kHz per channel */
#define PRINT_EVERY        250    /* samples -> 250 ms */
#define ZERO_SAMPLES       500
#define SETTLE_MS          300    /* skip inrush when averaging a step */
#define STALL_MAX_MS       1500   /* lab: never more than a second or two */

#define FREE_DUTY          0.5f   /* open loop, wheels off the ground */
#define STALL_DUTY         0.5f   /* keep modest: L298N ~2 A/ch; DD-15 max 80% */

/* Fault thresholds from the requirements (DD-17, DD-18). Nothing acts on
 * them here; the app only reports when a reading would have tripped one,
 * so the thresholds can be checked against real hardware. */
#define OPEN_MA            50     /* DD-17: below 0.05 A ...          */
#define OPEN_MIN_DUTY      0.2f   /*        ... while PWM >= 20% ...  */
#define OPEN_MS            200    /*        ... for 200 ms            */
#define OVERLOAD_MA        2000   /* DD-18: above 2.0 A ...           */
#define OVERLOAD_MS        20     /*        ... for 20 ms             */
#define NUM_MOTOR_CH       2      /* L and R; servo has no DD thresholds */
#define SWEEP_PERIOD_MS    2000

static const float filter_alphas[] = { 1.0f, 0.3f, 0.1f, 0.03f, 0.01f };
#define NUM_ALPHAS ARRAY_SIZE(filter_alphas)

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static int read_raw(const struct adc_dt_spec *ch, int16_t *raw)
{
    struct adc_sequence seq = {
        .buffer = raw,
        .buffer_size = sizeof(*raw),
    };

    adc_sequence_init_dt(ch, &seq);
    return adc_read_dt(ch, &seq);
}

/* Millivolts at the sensor output (before any divider). */
static float raw_to_sensor_mv(int i, int16_t raw)
{
    return raw * (ADC_VREF_MV / ADC_FULL) * cal[i].divider;
}

static float mv_to_ma(int i, float mv, float zero_mv)
{
    return (mv - zero_mv) * 1000.0f / cal[i].mv_per_a;
}

/* Current represented by one ADC count. */
static float lsb_ma(int i)
{
    return (ADC_VREF_MV / ADC_FULL) * cal[i].divider * 1000.0f / cal[i].mv_per_a;
}

/* Approximate time constant of the EMA filter: T * (1 - alpha) / alpha.
 * This is the lag you pay for the smoothing. */
static int filter_tau_ms(float alpha)
{
    return (int)((SAMPLE_PERIOD_US / 1000.0f) * (1.0f - alpha) / alpha + 0.5f);
}

static float absf(float x)
{
    return x < 0.0f ? -x : x;
}

/* ------------------------------------------------------------------ */
/* Outputs                                                            */
/* ------------------------------------------------------------------ */

static void motor_forward(float duty)
{
    gpio_pin_set(gpioc_dev, IN1_PIN, 1);
    gpio_pin_set(gpioa_dev, IN2_PIN, 0);
    duty = clampf(duty, 0.0f, 1.0f);
    pwm_set_cycles(motor_pwm, MOTOR_CHANNEL, PWM_PERIOD_CYCLES,
                   (uint32_t)(duty * PWM_PERIOD_CYCLES), 0);
}

static void motor_coast(void)
{
    pwm_set_cycles(motor_pwm, MOTOR_CHANNEL, PWM_PERIOD_CYCLES, 0, 0);
}

static void servo_write_us(int32_t us)
{
    pwm_set(servo_pwm, SERVO_CHANNEL, PWM_USEC(SERVO_PERIOD_US), PWM_USEC(us), 0);
}

static void servo_off(void)
{
    pwm_set(servo_pwm, SERVO_CHANNEL, PWM_USEC(SERVO_PERIOD_US), 0, 0);
}

/* Triangle wave between the steering limits. */
static int32_t sweep_us(int64_t t_ms)
{
    int32_t half = SWEEP_PERIOD_MS / 2;
    int32_t ph = (int32_t)(t_ms % SWEEP_PERIOD_MS);
    int32_t span = SERVO_RIGHT_US - SERVO_LEFT_US;

    if (ph < half) {
        return SERVO_LEFT_US + (span * ph) / half;
    }
    return SERVO_RIGHT_US - (span * (ph - half)) / half;
}

/* ------------------------------------------------------------------ */
/* Statistics                                                         */
/* ------------------------------------------------------------------ */

/* Noise over one print window, on unfiltered samples. */
struct window_stats {
    int16_t raw_min, raw_max;
    float sum, sum_sq;
    int n;
};

static void window_reset(struct window_stats *w)
{
    w->raw_min = INT16_MAX;
    w->raw_max = INT16_MIN;
    w->sum = 0.0f;
    w->sum_sq = 0.0f;
    w->n = 0;
}

static void window_add(struct window_stats *w, int16_t raw, float ma)
{
    w->raw_min = MIN(w->raw_min, raw);
    w->raw_max = MAX(w->raw_max, raw);
    w->sum += ma;
    w->sum_sq += ma * ma;
    w->n++;
}

/* Summary of one test step, on unfiltered samples. */
struct step_stats {
    float sum;          /* settled samples only */
    int n;
    float peak;         /* largest magnitude, including inrush */
};

static void step_reset(struct step_stats *s)
{
    s->sum = 0.0f;
    s->n = 0;
    s->peak = 0.0f;
}

static void step_add(struct step_stats *s, float ma, bool settled)
{
    if (settled) {
        s->sum += ma;
        s->n++;
    }
    if (absf(ma) > absf(s->peak)) {
        s->peak = ma;
    }
}

/* ------------------------------------------------------------------ */
/* Test sequence                                                      */
/* ------------------------------------------------------------------ */

enum step_kind { STEP_REST, STEP_FREE, STEP_STALL };

static const char *const step_names[] = { "REST", "FREE", "STALL" };
#define NUM_STEPS 3

static void print_step_summary(enum step_kind kind, const struct step_stats *st)
{
    printk("=== %s summary (mA): ", step_names[kind]);
    for (int i = 0; i < NUM_CH; i++) {
        int mean = st[i].n ? (int)(st[i].sum / st[i].n) : 0;
        printk("%s mean=%d peak=%d  ", cal[i].name, mean, (int)st[i].peak);
    }
    printk("\n");
}

static void announce_step(enum step_kind kind)
{
    switch (kind) {
    case STEP_REST:
        printk("step -> REST: motors coast, servo centered\n");
        break;
    case STEP_FREE:
        printk("step -> FREE: wheels OFF the ground, motors at %d%%, servo sweeping\n",
               (int)(FREE_DUTY * 100));
        break;
    case STEP_STALL:
        printk("step -> STALL: holding wheels + steering, cuts out after %d ms\n",
               STALL_MAX_MS);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

/* Consecutive-sample qualifier: reports once when a condition has held
 * for its full duration, re-arms when the condition clears. */
struct qualifier {
    int held_ms;
    bool reported;
};

static void qualify(struct qualifier *q, bool cond, int need_ms,
                    const char *what, const char *ch, float ma)
{
    if (!cond) {
        q->held_ms = 0;
        q->reported = false;
        return;
    }
    q->held_ms += SAMPLE_PERIOD_US / 1000;
    if (q->held_ms >= need_ms && !q->reported) {
        q->reported = true;
        printk("!! %s would trip on %s (now %d mA, held %d ms)\n",
               what, ch, (int)ma, q->held_ms);
    }
}

static float zero_mv[NUM_CH];

/* Average the sensors with everything off to find the zero-current output. */
static void calibrate_zero(void)
{
    float sum[NUM_CH] = { 0 };

    motor_coast();
    servo_off();
    k_msleep(200);

    for (int n = 0; n < ZERO_SAMPLES; n++) {
        for (int i = 0; i < NUM_CH; i++) {
            int16_t raw = 0;
            read_raw(&adc_chans[i], &raw);
            sum[i] += raw_to_sensor_mv(i, raw);
        }
        k_usleep(SAMPLE_PERIOD_US);
    }

    printk("zero (sensor mV):");
    for (int i = 0; i < NUM_CH; i++) {
        zero_mv[i] = sum[i] / ZERO_SAMPLES;
        printk(" %s=%d", cal[i].name, (int)zero_mv[i]);
    }
    printk("\n");
}

int main(void)
{
    for (int i = 0; i < NUM_CH; i++) {
        if (!adc_is_ready_dt(&adc_chans[i]) ||
            adc_channel_setup_dt(&adc_chans[i]) < 0) {
            printk("ERROR: ADC channel %s not ready\n", cal[i].name);
            return 0;
        }
    }
    if (!device_is_ready(motor_pwm) || !device_is_ready(servo_pwm) ||
        !device_is_ready(gpioa_dev) || !device_is_ready(gpioc_dev) ||
        !device_is_ready(console)) {
        printk("ERROR: device not ready\n");
        return 0;
    }

    gpio_pin_configure(gpioc_dev, IN1_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioa_dev, IN2_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioc_dev, BUTTON_PIN, GPIO_INPUT);
    motor_coast();

    printk("current_sensor_app ready. ADC 12 bit, ref %d mV\n", (int)ADC_VREF_MV);
    for (int i = 0; i < NUM_CH; i++) {
        printk("  %s: %d mV/A, divider x%d.%02d, 1 LSB = %d.%02d mA\n",
               cal[i].name, (int)cal[i].mv_per_a,
               (int)cal[i].divider, (int)(cal[i].divider * 100) % 100,
               (int)lsb_ma(i), (int)(lsb_ma(i) * 100) % 100);
    }

    calibrate_zero();

    enum step_kind step = STEP_REST;
    int64_t step_start_ms = k_uptime_get();
    bool stall_done = false;
    bool prev_pressed = false;
    int64_t last_button_ms = 0;

    size_t alpha_idx = 2;
    float filt[NUM_CH] = { 0 };
    struct window_stats win[NUM_CH];
    struct step_stats st[NUM_CH];
    int print_div = 0;
    struct qualifier open_q[NUM_MOTOR_CH] = { 0 };
    struct qualifier over_q[NUM_MOTOR_CH] = { 0 };

    for (int i = 0; i < NUM_CH; i++) {
        window_reset(&win[i]);
        step_reset(&st[i]);
    }

    printk("filter alpha=%d%% (tau ~%d ms). keys: f filter, z re-zero\n",
           (int)(filter_alphas[alpha_idx] * 100),
           filter_tau_ms(filter_alphas[alpha_idx]));
    announce_step(step);

    struct k_timer tick;
    k_timer_init(&tick, NULL, NULL);
    k_timer_start(&tick, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

    while (1) {
        k_timer_status_sync(&tick);
        int64_t now_ms = k_uptime_get();
        int64_t in_step_ms = now_ms - step_start_ms;

        /* Serial keys */
        unsigned char c;
        if (uart_poll_in(console, &c) == 0) {
            if (c == 'f') {
                alpha_idx = (alpha_idx + 1) % NUM_ALPHAS;
                printk("filter alpha=%d%% (tau ~%d ms)\n",
                       (int)(filter_alphas[alpha_idx] * 100),
                       filter_tau_ms(filter_alphas[alpha_idx]));
            } else if (c == 'z') {
                if (step == STEP_REST) {
                    calibrate_zero();
                    for (int i = 0; i < NUM_CH; i++) {
                        filt[i] = 0.0f;
                    }
                } else {
                    printk("re-zero only in REST\n");
                }
            }
        }

        /* Button: finish the current step, start the next.
         * Sampled every 20 ms, which debounces it. */
        bool pressed = prev_pressed;
        if (now_ms - last_button_ms >= 20) {
            last_button_ms = now_ms;
            pressed = (gpio_pin_get(gpioc_dev, BUTTON_PIN) == 0);
        }
        if (pressed && !prev_pressed) {
            if (!(step == STEP_STALL && stall_done)) {
                print_step_summary(step, st);
            }
            step = (step + 1) % NUM_STEPS;
            step_start_ms = now_ms;
            in_step_ms = 0;
            stall_done = false;
            for (int i = 0; i < NUM_CH; i++) {
                step_reset(&st[i]);
            }
            announce_step(step);
        }
        prev_pressed = pressed;

        /* Stall time limit */
        if (step == STEP_STALL && !stall_done && in_step_ms >= STALL_MAX_MS) {
            stall_done = true;
            motor_coast();
            servo_write_us(SERVO_CENTER_US);
            printk("stall time limit reached, outputs off - let go\n");
            print_step_summary(step, st);
        }

        /* Outputs */
        float duty = 0.0f;
        switch (step) {
        case STEP_REST:
            motor_coast();
            servo_write_us(SERVO_CENTER_US);
            break;
        case STEP_FREE:
            duty = FREE_DUTY;
            motor_forward(duty);
            servo_write_us(sweep_us(in_step_ms));
            break;
        case STEP_STALL:
            if (!stall_done) {
                duty = STALL_DUTY;
                motor_forward(duty);
                servo_write_us(SERVO_LEFT_US);
            }
            break;
        }

        /* Measure */
        bool settled = in_step_ms >= SETTLE_MS;
        bool recording = !(step == STEP_STALL && stall_done);
        float alpha = filter_alphas[alpha_idx];

        for (int i = 0; i < NUM_CH; i++) {
            int16_t raw = 0;
            if (read_raw(&adc_chans[i], &raw) < 0) {
                continue;
            }
            float ma = mv_to_ma(i, raw_to_sensor_mv(i, raw), zero_mv[i]);

            filt[i] += alpha * (ma - filt[i]);
            window_add(&win[i], raw, ma);
            if (recording) {
                step_add(&st[i], ma, settled);
            }
            if (i < NUM_MOTOR_CH) {
                qualify(&open_q[i], duty >= OPEN_MIN_DUTY && absf(ma) < OPEN_MA,
                        OPEN_MS, "OPEN CIRCUIT (DD-17)", cal[i].name, ma);
                qualify(&over_q[i], absf(ma) > OVERLOAD_MA,
                        OVERLOAD_MS, "OVERLOAD (DD-18)", cal[i].name, ma);
            }
        }

        /* Print every PRINT_EVERY samples: filtered mA, window mean,
         * peak-to-peak noise (counts and mA), std dev. Integers only. */
        if (++print_div >= PRINT_EVERY) {
            print_div = 0;
            for (int i = 0; i < NUM_CH; i++) {
                struct window_stats *w = &win[i];
                float mean = w->n ? w->sum / w->n : 0.0f;
                float var = w->n ? w->sum_sq / w->n - mean * mean : 0.0f;
                int pp = w->n ? w->raw_max - w->raw_min : 0;

                printk("%s f=%d avg=%d pp=%dLSB/%dmA sd=%d | ",
                       cal[i].name, (int)filt[i], (int)mean,
                       pp, (int)(pp * lsb_ma(i)), (int)__builtin_sqrtf(MAX(var, 0.0f)));
                window_reset(w);
            }
            printk("\n");
        }
    }

    return 0;
}
