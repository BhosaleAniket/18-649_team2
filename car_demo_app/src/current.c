/*
 * current.c - ACS712-05B current sensors (left motor, right motor, servo)
 *
 * One low-priority thread samples the three ADC channels every 1 ms, removes
 * each sensor's zero-current offset, and keeps an exponential moving average
 * (time constant about 100 ms) of the current. The averaged magnitude in mA is
 * published with atomics for the status frame and the monitor.
 *
 * Wiring (see docs/lab2_status_report.md):
 *   PA0 (ADC1_IN0) left motor, PA4 (ADC1_IN4) right motor, PB0 (ADC1_IN8) servo.
 *   Each ACS712 OUT goes through a 47k/47k divider to the pin, so the sensor
 *   voltage is the pin voltage times CURRENT_DIVIDER.
 *
 * Zero: measured once at boot, averaged over ZERO_SAMPLES. Must happen with
 * the sensors powered and the motors idle. If the result is out of range the
 * sensor rail is probably not up yet, so it retries every second and reports
 * 0 mA until a sane zero is found.
 *
 * Only the magnitude is reported, so the direction the current flows through
 * a sensor does not matter.
 *
 * The thread only reads the ADC and updates atomics; it never blocks anything
 * else, and all printing stays in the monitor.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/atomic.h>

#include "current.h"
#include "config.h"

#define USER_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_chans[] = {
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, i_left),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, i_right),
    ADC_DT_SPEC_GET_BY_NAME(USER_NODE, i_servo),
};
#define NUM_CH ARRAY_SIZE(adc_chans)

/* ---- Sensor and divider constants ---- */
#define ADC_VREF_MV        3300.0f      /* VREF+ = VDDA = 3.3 V on the Nucleo */
#define ADC_FULL           4096.0f      /* 12 bit */
#define CURRENT_DIVIDER    2.0f         /* (47k + 47k) / 47k */
#define SENSOR_MV_PER_A    185.0f       /* ACS712-05B */

/* ---- Timing and filtering ---- */
#define SAMPLE_PERIOD_US   1000         /* 1 kHz per channel */
#define FILTER_ALPHA       0.01f        /* tau = 1 ms * (1 - a) / a, about 99 ms */
#define ZERO_SETTLE_MS     300          /* let rails and outputs settle at boot */
#define ZERO_SAMPLES       500
#define ZERO_RETRY_MS      1000
#define ZERO_MIN_MV        2000.0f      /* sane zero-current range at the sensor */
#define ZERO_MAX_MV        3000.0f

static float zero_mv[NUM_CH];
static float filt_ma[NUM_CH];

static atomic_t pub_ma[NUM_CH];
static atomic_t zero_valid;

void current_get(struct current_readings *out)
{
    out->left_ma  = (uint16_t)atomic_get(&pub_ma[0]);
    out->right_ma = (uint16_t)atomic_get(&pub_ma[1]);
    out->servo_ma = (uint16_t)atomic_get(&pub_ma[2]);
}

bool current_is_valid(void)
{
    return atomic_get(&zero_valid) != 0;
}

static int read_raw(int ch, int16_t *raw)
{
    struct adc_sequence seq = {
        .buffer = raw,
        .buffer_size = sizeof(*raw),
    };

    adc_sequence_init_dt(&adc_chans[ch], &seq);
    return adc_read_dt(&adc_chans[ch], &seq);
}

/* Millivolts at the sensor output (before the divider). */
static float raw_to_sensor_mv(int16_t raw)
{
    return raw * (ADC_VREF_MV / ADC_FULL) * CURRENT_DIVIDER;
}

/* Average the sensors with everything idle. True if all three look sane. */
static bool calibrate_zero(void)
{
    float sum[NUM_CH] = { 0 };

    k_msleep(ZERO_SETTLE_MS);

    for (int n = 0; n < ZERO_SAMPLES; n++) {
        for (int i = 0; i < NUM_CH; i++) {
            int16_t raw = 0;
            if (read_raw(i, &raw) < 0) {
                return false;
            }
            sum[i] += raw_to_sensor_mv(raw);
        }
        k_usleep(SAMPLE_PERIOD_US);
    }

    for (int i = 0; i < NUM_CH; i++) {
        float z = sum[i] / ZERO_SAMPLES;
        if (z < ZERO_MIN_MV || z > ZERO_MAX_MV) {
            return false;
        }
        zero_mv[i] = z;
    }
    return true;
}

K_THREAD_STACK_DEFINE(current_stack, STACK_CURRENT);
static struct k_thread current_thread_data;

static void current_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* Retry until the sensors give a sane zero (rail may not be up yet) */
    while (!calibrate_zero()) {
        k_msleep(ZERO_RETRY_MS);
    }
    atomic_set(&zero_valid, 1);

    struct k_timer tick;
    k_timer_init(&tick, NULL, NULL);
    k_timer_start(&tick, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

    while (1) {
        /* Expiries are scheduled from the previous one, so no drift */
        k_timer_status_sync(&tick);

        for (int i = 0; i < NUM_CH; i++) {
            int16_t raw = 0;
            if (read_raw(i, &raw) < 0) {
                continue;
            }
            float ma = (raw_to_sensor_mv(raw) - zero_mv[i]) * 1000.0f / SENSOR_MV_PER_A;
            filt_ma[i] += FILTER_ALPHA * (ma - filt_ma[i]);

            float mag = filt_ma[i] < 0.0f ? -filt_ma[i] : filt_ma[i];
            if (mag > 65535.0f) {
                mag = 65535.0f;
            }
            atomic_set(&pub_ma[i], (atomic_val_t)(mag + 0.5f));
        }
    }
}

int current_init(void)
{
    for (int i = 0; i < NUM_CH; i++) {
        if (!adc_is_ready_dt(&adc_chans[i]) ||
            adc_channel_setup_dt(&adc_chans[i]) < 0) {
            return -ENODEV;
        }
    }

    k_thread_create(&current_thread_data, current_stack,
                    K_THREAD_STACK_SIZEOF(current_stack),
                    current_thread, NULL, NULL, NULL,
                    PRIO_CURRENT, 0, K_NO_WAIT);
    k_thread_name_set(&current_thread_data, "current");
    return 0;
}
