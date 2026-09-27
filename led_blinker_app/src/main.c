/*
 * blinker_app - turn signals and hazards (Nucleo-F401RE)
 *
 * Modes:
 *   LEFT    FL + RL blink at 1 Hz, 50% duty  (500 ms on / 500 ms off)
 *   RIGHT   FR + RR blink at 1 Hz, 50% duty
 *   HAZARD  all four blink together at 2 Hz, 50% duty (250 ms on / 250 ms off)
 *   OFF     all dark
 * Selecting one side replaces the other (one side cancels the other).
 *
 * Timing design:
 *   One periodic k_timer ticks every 250 ms, the half-period of the fastest
 *   pattern. Hazards toggle every tick; turn signals toggle every 2 ticks.
 *   A periodic k_timer schedules each expiry from the previous one, not from
 *   when the handler happened to run, so the period does not drift even if
 *   other work delays the handler slightly. Its accuracy is the accuracy of
 *   the system clock, which on this board comes from the ST-LINK's 8 MHz
 *   crystal (HSE).
 *
 *   The timer handler runs in interrupt context: it only reads the mode and
 *   writes GPIO pins. No printing, no blocking, no allocation.
 *
 *   Front and rear of the same side are written back to back in the same
 *   handler, so they switch within microseconds of each other.
 *
 * Mode changes:
 *   The thread stops the timer, updates the mode, turns the pattern's LEDs on
 *   immediately, and restarts the timer. A new mode therefore shows up at once
 *   rather than at the next tick, and each pattern starts on a clean "on" phase.
 *
 * Inputs (bench testing only; the final system uses wheel buttons):
 *   User button (PC13)   cycles OFF -> LEFT -> RIGHT -> HAZARD -> OFF
 *   Serial keys          o = off, l = left, r = right, h = hazard
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>

/* ------------------------------------------------------------------ */
/* Hardware                                                           */
/* ------------------------------------------------------------------ */

static const struct gpio_dt_spec led_fl = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_fl), gpios);
static const struct gpio_dt_spec led_fr = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_fr), gpios);
static const struct gpio_dt_spec led_rl = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_rl), gpios);
static const struct gpio_dt_spec led_rr = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_rr), gpios);

static const struct device *gpioc_dev = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *console   = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define BUTTON_PIN   13     /* PC13, active low */

/* ------------------------------------------------------------------ */
/* Timing                                                             */
/* ------------------------------------------------------------------ */

#define TICK_MS      250    /* half-period of the 2 Hz hazard pattern */
#define POLL_MS      10     /* input polling period */

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

enum blink_mode { MODE_OFF, MODE_LEFT, MODE_RIGHT, MODE_HAZARD, NUM_MODES };

static const char *const mode_names[NUM_MODES] = {
    "OFF", "LEFT", "RIGHT", "HAZARD"
};

/* Written by the thread, read by the timer ISR. atomic_t makes each
 * read and write indivisible, which volatile alone would not guarantee. */
static atomic_t mode = ATOMIC_INIT(MODE_OFF);

/* Tick counter. Only the timer handler touches it while the timer runs;
 * the thread resets it only while the timer is stopped. */
static uint32_t ticks;

/* ------------------------------------------------------------------ */
/* Outputs                                                            */
/* ------------------------------------------------------------------ */

static void apply_outputs(enum blink_mode m, bool on)
{
    bool left  = on && (m == MODE_LEFT  || m == MODE_HAZARD);
    bool right = on && (m == MODE_RIGHT || m == MODE_HAZARD);

    /* Same-side front and rear written back to back */
    gpio_pin_set_dt(&led_fl, left);
    gpio_pin_set_dt(&led_rl, left);
    gpio_pin_set_dt(&led_fr, right);
    gpio_pin_set_dt(&led_rr, right);
}

/* ------------------------------------------------------------------ */
/* Timer handler - interrupt context, keep it short                   */
/* ------------------------------------------------------------------ */

static void blink_tick(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    enum blink_mode m = (enum blink_mode)atomic_get(&mode);
    ticks++;

    bool on;
    if (m == MODE_HAZARD) {
        on = (ticks % 2) == 0;          /* toggle every 250 ms -> 2 Hz */
    } else {
        on = ((ticks / 2) % 2) == 0;    /* toggle every 500 ms -> 1 Hz */
    }

    apply_outputs(m, on);
}

K_TIMER_DEFINE(blink_timer, blink_tick, NULL);

/* ------------------------------------------------------------------ */
/* Mode changes - thread context                                      */
/* ------------------------------------------------------------------ */

static void set_mode(enum blink_mode m)
{
    k_timer_stop(&blink_timer);         /* handler cannot run past this point */

    atomic_set(&mode, m);
    ticks = 0;

    if (m == MODE_OFF) {
        apply_outputs(m, false);
    } else {
        apply_outputs(m, true);         /* visible immediately, starts "on" */
        k_timer_start(&blink_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    }

    printk("[%u ms] mode -> %s\n", (uint32_t)k_uptime_get(), mode_names[m]);
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

static bool led_ready(const struct gpio_dt_spec *led, const char *name)
{
    if (!gpio_is_ready_dt(led)) {
        printk("ERROR: %s not ready\n", name);
        return false;
    }
    gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
    return true;
}

int main(void)
{
    if (!led_ready(&led_fl, "FL") || !led_ready(&led_fr, "FR") ||
        !led_ready(&led_rl, "RL") || !led_ready(&led_rr, "RR") ||
        !device_is_ready(gpioc_dev) || !device_is_ready(console)) {
        printk("ERROR: device not ready\n");
        return 0;
    }

    gpio_pin_configure(gpioc_dev, BUTTON_PIN, GPIO_INPUT);

    printk("blinker_app ready\n");
    printk("button cycles modes | keys: o off, l left, r right, h hazard\n");
    set_mode(MODE_OFF);

    bool prev_pressed = false;

    while (1) {
        enum blink_mode current = (enum blink_mode)atomic_get(&mode);

        /* Serial keys */
        unsigned char c;
        if (uart_poll_in(console, &c) == 0) {
            switch (c) {
            case 'o': set_mode(MODE_OFF);    break;
            case 'l': set_mode(MODE_LEFT);   break;
            case 'r': set_mode(MODE_RIGHT);  break;
            case 'h': set_mode(MODE_HAZARD); break;
            default: break;
            }
        }

        /* Button: cycle modes on each press (10 ms polling debounces it) */
        bool pressed = (gpio_pin_get(gpioc_dev, BUTTON_PIN) == 0);
        if (pressed && !prev_pressed) {
            set_mode((enum blink_mode)((current + 1) % NUM_MODES));
        }
        prev_pressed = pressed;

        k_msleep(POLL_MS);
    }

    return 0;
}