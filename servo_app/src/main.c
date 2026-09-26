/*
 * servo_app - steering servo calibration and test (Nucleo-F401RE)
 *
 * Servo: Hiwonder LD-1501MG
 *   Signal: 20 ms period, pulse 500-2500 us (0-180 deg), 1500 us center
 *   Power:  6-8.4 V from a separate buck converter, NOT the Nucleo
 *
 * Pins:
 *   PB8  TIM10_CH1 -> servo signal
 *   PC13 GPIO      <- user button (active low), steps the sanity test
 *
 * Serial console (ST-LINK COM port, 115200) keys, for calibration:
 *   a / d   pulse -10 / +10 us
 *   A / D   pulse -50 / +50 us
 *   c       pulse = 1500 us (center)
 *   t       leave manual mode, back to the button test
 *
 * Calibration: step toward each end until the linkage stops or the
 * servo buzzes, back off 20-30 us, and record the three values below.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

/* ------------------------------------------------------------------ */
/* Hardware                                                           */
/* ------------------------------------------------------------------ */

static const struct device *servo_pwm = DEVICE_DT_GET(DT_NODELABEL(pwm10));
static const struct device *gpioc_dev = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *console   = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define SERVO_CHANNEL    1        /* TIM10_CH1 */
#define BUTTON_PIN       13       /* PC13, active low */

/* ------------------------------------------------------------------ */
/* Servo limits                                                       */
/* ------------------------------------------------------------------ */

#define SERVO_PERIOD_US  20000

/* Electrical range of the LD-1501MG. Nothing outside this is ever sent. */
#define SERVO_ELEC_MIN_US  500
#define SERVO_ELEC_MAX_US  2500

/* Calibrated mechanical limits of the chassis steering linkage.
 * PLACEHOLDERS - replace with your measured values.
 * LEFT may be larger or smaller than RIGHT; the mapping handles either. */
#define SERVO_LEFT_US    1300
#define SERVO_CENTER_US  1500
#define SERVO_RIGHT_US   1700

#define LIMIT_LO  (SERVO_LEFT_US < SERVO_RIGHT_US ? SERVO_LEFT_US : SERVO_RIGHT_US)
#define LIMIT_HI  (SERVO_LEFT_US < SERVO_RIGHT_US ? SERVO_RIGHT_US : SERVO_LEFT_US)

#define LOOP_PERIOD_MS   20

/* ------------------------------------------------------------------ */
/* Servo output                                                       */
/* ------------------------------------------------------------------ */

static int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Raw write, clamped only to the servo's electrical range (calibration). */
static void servo_write_raw_us(int32_t us)
{
    us = clamp_i32(us, SERVO_ELEC_MIN_US, SERVO_ELEC_MAX_US);
    pwm_set(servo_pwm, SERVO_CHANNEL, PWM_USEC(SERVO_PERIOD_US), PWM_USEC(us), 0);
}

/* Normal write, clamped to the calibrated mechanical limits. */
static void servo_write_us(int32_t us)
{
    servo_write_raw_us(clamp_i32(us, LIMIT_LO, LIMIT_HI));
}

/*
 * Documented, monotonic mapping from the cockpit wheel to the servo:
 *   steer = -32768 (full left)  -> SERVO_LEFT_US
 *   steer =      0 (center)     -> SERVO_CENTER_US
 *   steer = +32767 (full right) -> SERVO_RIGHT_US
 * Piecewise linear on each side of center, so straight-ahead is exact
 * even if center is not midway between the two ends.
 */
static int32_t steer_to_us(int16_t steer)
{
    if (steer >= 0) {
        return SERVO_CENTER_US +
               ((int32_t)steer * (SERVO_RIGHT_US - SERVO_CENTER_US)) / 32767;
    }
    return SERVO_CENTER_US +
           ((int32_t)steer * (SERVO_CENTER_US - SERVO_LEFT_US)) / 32768;
}

/* ------------------------------------------------------------------ */
/* Sanity test sequence (user button steps through it)                */
/* ------------------------------------------------------------------ */

struct test_step {
    const char *name;
    int16_t steer;
};

static const struct test_step test_steps[] = {
    { "CENTER",     0      },
    { "FULL LEFT",  -32768 },
    { "CENTER",     0      },
    { "FULL RIGHT", 32767  },
};
#define NUM_STEPS (sizeof(test_steps) / sizeof(test_steps[0]))

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

int main(void)
{
    if (!device_is_ready(servo_pwm) || !device_is_ready(gpioc_dev) ||
        !device_is_ready(console)) {
        printk("ERROR: device not ready\n");
        return 0;
    }

    gpio_pin_configure(gpioc_dev, BUTTON_PIN, GPIO_INPUT);

    size_t step = 0;
    bool prev_pressed = false;

    bool manual = false;                /* calibration mode */
    int32_t manual_us = SERVO_CENTER_US;

    servo_write_us(steer_to_us(test_steps[step].steer));
    printk("servo_app ready. limits L=%d C=%d R=%d us\n",
           SERVO_LEFT_US, SERVO_CENTER_US, SERVO_RIGHT_US);
    printk("button: step test | keys a/d A/D c t: calibrate\n");
    printk("test -> %s (%d us)\n", test_steps[step].name,
           (int)steer_to_us(test_steps[step].steer));

    while (1) {
        /* Serial keys: calibration mode */
        unsigned char c;
        if (uart_poll_in(console, &c) == 0) {
            int32_t delta = 0;

            switch (c) {
            case 'a': delta = -10; break;
            case 'd': delta = +10; break;
            case 'A': delta = -50; break;
            case 'D': delta = +50; break;
            case 'c': manual = true; manual_us = 1500; break;
            case 't':
                manual = false;
                printk("back to test -> %s\n", test_steps[step].name);
                break;
            default: break;
            }

            if (delta != 0) {
                if (!manual) {
                    manual = true;
                    manual_us = steer_to_us(test_steps[step].steer);
                }
                manual_us = clamp_i32(manual_us + delta,
                                      SERVO_ELEC_MIN_US, SERVO_ELEC_MAX_US);
            }
            if (manual && c != 't') {
                printk("manual: %d us\n", (int)manual_us);
            }
        }

        /* Button: step the sanity test */
        bool pressed = (gpio_pin_get(gpioc_dev, BUTTON_PIN) == 0);
        if (pressed && !prev_pressed) {
            manual = false;
            step = (step + 1) % NUM_STEPS;
            printk("test -> %s (%d us)\n", test_steps[step].name,
                   (int)steer_to_us(test_steps[step].steer));
        }
        prev_pressed = pressed;

        /* Output */
        if (manual) {
            servo_write_raw_us(manual_us);
        } else {
            servo_write_us(steer_to_us(test_steps[step].steer));
        }

        k_msleep(LOOP_PERIOD_MS);
    }

    return 0;
}