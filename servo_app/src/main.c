/*
 * servo_app - steering servo calibration and test (Nucleo-F401RE)
 *
 * Servo: Hiwonder LD-1501MG
 *   Signal: 20 ms period, pulse 500-2500 us = 0-180 deg (servo shaft)
 *   Power:  6-8.4 V from a separate buck converter, NOT the Nucleo
 *
 * All calibration is in SERVO SHAFT DEGREES (90 deg = servo center).
 * These are not front-wheel angles; the linkage changes the ratio.
 * Conversion: us = 500 + deg * (2000 / 180), about 11.1 us per degree.
 *
 * Pins:
 *   PB8  TIM10_CH1 -> servo signal
 *   PC13 GPIO      <- user button (active low), steps the sanity test
 *
 * Serial console (ST-LINK COM port, 115200) keys, for calibration:
 *   a / d   -1 / +1 degree
 *   A / D   -5 / +5 degrees
 *   c       go to the calibrated center (SERVO_CENTER_DEG)
 *   t       leave manual mode, back to the button test
 *
 * Calibration: step toward each end until the linkage stops or the
 * servo buzzes, back off 2-3 degrees, and record the three values below.
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
#define LOOP_PERIOD_MS   20

/* ------------------------------------------------------------------ */
/* Servo signal                                                       */
/* ------------------------------------------------------------------ */

#define SERVO_PERIOD_US    20000
#define SERVO_MIN_US       500      /* 0 deg   */
#define SERVO_MAX_US       2500     /* 180 deg */
#define SERVO_RANGE_DEG    180.0f

/* ------------------------------------------------------------------ */
/* Calibration - servo shaft degrees                                  */
/* PLACEHOLDERS: replace with your measured values.                   */
/* LEFT may be larger or smaller than RIGHT; the mapping handles it.  */
/* ------------------------------------------------------------------ */

#define SERVO_LEFT_DEG     72.0f
#define SERVO_CENTER_DEG   90.0f
#define SERVO_RIGHT_DEG    108.0f

/* Calibration key step sizes */
#define STEP_FINE_DEG      1.0f
#define STEP_COARSE_DEG    5.0f

#define LIMIT_LO_DEG (SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_LEFT_DEG : SERVO_RIGHT_DEG)
#define LIMIT_HI_DEG (SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_RIGHT_DEG : SERVO_LEFT_DEG)

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static int32_t deg_to_us(float deg)
{
    float us = SERVO_MIN_US + deg * ((SERVO_MAX_US - SERVO_MIN_US) / SERVO_RANGE_DEG);
    return (int32_t)(us + 0.5f);
}

/* Print a float with one decimal place, without needing float printf */
static void print_deg(const char *label, float deg)
{
    int tenths = (int)(deg * 10.0f + (deg >= 0.0f ? 0.5f : -0.5f));
    int whole = tenths / 10;
    int frac = tenths % 10;
    if (frac < 0) {
        frac = -frac;
    }
    printk("%s%d.%d deg (%d us)\n", label, whole, frac, (int)deg_to_us(deg));
}

/* ------------------------------------------------------------------ */
/* Servo output                                                       */
/* ------------------------------------------------------------------ */

/* Raw write, clamped only to the servo's full range (calibration). */
static void servo_write_raw_deg(float deg)
{
    deg = clampf(deg, 0.0f, SERVO_RANGE_DEG);
    pwm_set(servo_pwm, SERVO_CHANNEL,
            PWM_USEC(SERVO_PERIOD_US), PWM_USEC(deg_to_us(deg)), 0);
}

/* Normal write, clamped to the calibrated mechanical limits. */
static void servo_write_deg(float deg)
{
    servo_write_raw_deg(clampf(deg, LIMIT_LO_DEG, LIMIT_HI_DEG));
}

/*
 * Documented, monotonic mapping from the cockpit wheel to the servo:
 *   steer = -32768 (full left)  -> SERVO_LEFT_DEG
 *   steer =      0 (center)     -> SERVO_CENTER_DEG
 *   steer = +32767 (full right) -> SERVO_RIGHT_DEG
 * Piecewise linear on each side of center, so straight-ahead is exact
 * even if center is not midway between the two ends.
 */
static float steer_to_deg(int16_t steer)
{
    if (steer >= 0) {
        return SERVO_CENTER_DEG +
               (steer / 32767.0f) * (SERVO_RIGHT_DEG - SERVO_CENTER_DEG);
    }
    return SERVO_CENTER_DEG +
           (steer / 32768.0f) * (SERVO_CENTER_DEG - SERVO_LEFT_DEG);
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

static void print_test_step(size_t step)
{
    printk("test -> %s: ", test_steps[step].name);
    print_deg("", steer_to_deg(test_steps[step].steer));
}

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
    float manual_deg = SERVO_CENTER_DEG;

    servo_write_deg(steer_to_deg(test_steps[step].steer));

    printk("servo_app ready. calibrated limits:\n");
    print_deg("  left   ", SERVO_LEFT_DEG);
    print_deg("  center ", SERVO_CENTER_DEG);
    print_deg("  right  ", SERVO_RIGHT_DEG);
    printk("button: step test | keys a/d (1 deg) A/D (5 deg) c t\n");
    print_test_step(step);

    while (1) {
        /* Serial keys: calibration mode */
        unsigned char c;
        if (uart_poll_in(console, &c) == 0) {
            float delta = 0.0f;
            bool changed = false;

            switch (c) {
            case 'a': delta = -STEP_FINE_DEG;   break;
            case 'd': delta = +STEP_FINE_DEG;   break;
            case 'A': delta = -STEP_COARSE_DEG; break;
            case 'D': delta = +STEP_COARSE_DEG; break;
            case 'c':
                manual = true;
                manual_deg = SERVO_CENTER_DEG;
                changed = true;
                break;
            case 't':
                manual = false;
                printk("back to ");
                print_test_step(step);
                break;
            default:
                break;
            }

            if (delta != 0.0f) {
                if (!manual) {
                    manual = true;
                    manual_deg = steer_to_deg(test_steps[step].steer);
                }
                manual_deg = clampf(manual_deg + delta, 0.0f, SERVO_RANGE_DEG);
                changed = true;
            }
            if (changed) {
                print_deg("manual: ", manual_deg);
            }
        }

        /* Button: step the sanity test */
        bool pressed = (gpio_pin_get(gpioc_dev, BUTTON_PIN) == 0);
        if (pressed && !prev_pressed) {
            manual = false;
            step = (step + 1) % NUM_STEPS;
            print_test_step(step);
        }
        prev_pressed = pressed;

        /* Output */
        if (manual) {
            servo_write_raw_deg(manual_deg);
        } else {
            servo_write_deg(steer_to_deg(test_steps[step].steer));
        }

        k_msleep(LOOP_PERIOD_MS);
    }

    return 0;
}