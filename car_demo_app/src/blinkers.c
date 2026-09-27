/*
 * blinkers.c - turn signals and hazards
 *
 * Two pieces, deliberately separate:
 *   blink output  - periodic k_timer ISR, 250 ms. Only reads the pattern
 *                   and writes pins, so timing is set by the timer, not by
 *                   when a thread gets to run. 1 Hz turn, 2 Hz hazard, 50%.
 *   blinker logic - low-priority thread, 20 ms. Button edges, self-cancel,
 *                   hazard selection. Its 100 ms budget is far looser than
 *                   the control thread's 2 ms, hence the lower priority.
 *
 * Turn signal rules:
 *   - A press on the left or right button (debounced) selects that side;
 *     one side cancels the other; pressing the active side again turns it off.
 *   - Self-cancel: once the wheel turns past TURN_THRESHOLD toward the active
 *     side (armed), returning back past (TURN_THRESHOLD - TURN_HYSTERESIS)
 *     cancels the signal.
 *   - Error state: hazards override the display. The turn state is kept
 *     underneath and reappears when the error clears.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "blinkers.h"
#include "link.h"
#include "link_proto.h"
#include "control.h"
#include "config.h"

static const struct gpio_dt_spec led_fl = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_fl), gpios);
static const struct gpio_dt_spec led_fr = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_fr), gpios);
static const struct gpio_dt_spec led_rl = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_rl), gpios);
static const struct gpio_dt_spec led_rr = GPIO_DT_SPEC_GET(DT_NODELABEL(blinker_rr), gpios);

/* Steering thresholds in wheel units, as fractions of the chassis full lock
 * (STEER_FULL_SCALE in config.h), so they stay sensible if that changes. */
#define TURN_THRESHOLD   (STEER_FULL_SCALE / 4)     /* 25% of full lock */
#define TURN_HYSTERESIS  (STEER_FULL_SCALE / 20)    /* 5% */

/* ------------------------------------------------------------------ */
/* Blink output - timer ISR                                           */
/* ------------------------------------------------------------------ */

enum blink_mode { MODE_OFF, MODE_LEFT, MODE_RIGHT, MODE_HAZARD };

static atomic_t display_mode = ATOMIC_INIT(MODE_OFF);
static uint32_t ticks;

static void apply_outputs(enum blink_mode m, bool on)
{
    bool left  = on && (m == MODE_LEFT  || m == MODE_HAZARD);
    bool right = on && (m == MODE_RIGHT || m == MODE_HAZARD);

    gpio_pin_set_dt(&led_fl, left);
    gpio_pin_set_dt(&led_rl, left);
    gpio_pin_set_dt(&led_fr, right);
    gpio_pin_set_dt(&led_rr, right);
}

static void blink_tick(struct k_timer *t)
{
    ARG_UNUSED(t);
    enum blink_mode m = (enum blink_mode)atomic_get(&display_mode);
    ticks++;
    bool on = (m == MODE_HAZARD) ? ((ticks % 2) == 0) : (((ticks / 2) % 2) == 0);
    apply_outputs(m, on);
}

K_TIMER_DEFINE(blink_timer, blink_tick, NULL);

static void set_display(enum blink_mode m)
{
    if ((enum blink_mode)atomic_get(&display_mode) == m) {
        return;                         /* unchanged: keep the phase */
    }
    k_timer_stop(&blink_timer);
    atomic_set(&display_mode, m);
    ticks = 0;
    if (m == MODE_OFF) {
        apply_outputs(m, false);
    } else {
        apply_outputs(m, true);         /* visible at once, starts "on" */
        k_timer_start(&blink_timer, K_MSEC(BLINK_TICK_MS), K_MSEC(BLINK_TICK_MS));
    }
}

/* ------------------------------------------------------------------ */
/* Blinker logic - thread                                             */
/* ------------------------------------------------------------------ */

enum turn_state { TURN_OFF, TURN_LEFT, TURN_RIGHT };
static const char *const turn_names[] = { "off", "left", "right" };
static atomic_t turn_pub = ATOMIC_INIT(TURN_OFF);

const char *blinkers_turn_name(void)
{
    return turn_names[atomic_get(&turn_pub)];
}

/* Debounced rising-edge detector for a button level */
struct button {
    bool level;
    int64_t change_ms;
};

static bool button_pressed(struct button *b, bool level, int64_t now)
{
    if (level != b->level && (now - b->change_ms) >= BUTTON_DEBOUNCE_MS) {
        b->level = level;
        b->change_ms = now;
        return level;                   /* true only on the press edge */
    }
    return false;
}

K_THREAD_STACK_DEFINE(blinker_stack, STACK_BLINKER);
static struct k_thread blinker_thread_data;

static void blinker_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    enum turn_state turn = TURN_OFF;
    bool armed = false;
    struct button left_btn  = { .change_ms = -100000 };
    struct button right_btn = { .change_ms = -100000 };

    while (1) {
        int64_t now = k_uptime_get();
        struct link_cmd cmd;
        link_get_command(&cmd);

        if (cmd.valid) {
            /* Button presses */
            if (button_pressed(&left_btn, (cmd.flags & CMD_FLAG_LEFT) != 0, now)) {
                turn = (turn == TURN_LEFT) ? TURN_OFF : TURN_LEFT;
                armed = false;
            }
            if (button_pressed(&right_btn, (cmd.flags & CMD_FLAG_RIGHT) != 0, now)) {
                turn = (turn == TURN_RIGHT) ? TURN_OFF : TURN_RIGHT;
                armed = false;
            }

            /* Self-cancel */
            int32_t s = cmd.steer;
            if (turn == TURN_LEFT) {
                if (!armed && s <= -TURN_THRESHOLD) {
                    armed = true;
                } else if (armed && s > -(TURN_THRESHOLD - TURN_HYSTERESIS)) {
                    turn = TURN_OFF;
                    armed = false;
                }
            } else if (turn == TURN_RIGHT) {
                if (!armed && s >= TURN_THRESHOLD) {
                    armed = true;
                } else if (armed && s < (TURN_THRESHOLD - TURN_HYSTERESIS)) {
                    turn = TURN_OFF;
                    armed = false;
                }
            }
        }
        atomic_set(&turn_pub, turn);

        /* Display: hazards in the error state, otherwise the turn signal */
        if (control_get_state() == ST_STATE_ERROR) {
            set_display(MODE_HAZARD);
        } else if (turn == TURN_LEFT) {
            set_display(MODE_LEFT);
        } else if (turn == TURN_RIGHT) {
            set_display(MODE_RIGHT);
        } else {
            set_display(MODE_OFF);
        }

        k_msleep(BLINKER_PERIOD_MS);
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

int blinkers_init(void)
{
    const struct gpio_dt_spec *leds[] = { &led_fl, &led_fr, &led_rl, &led_rr };
    for (int i = 0; i < 4; i++) {
        if (!gpio_is_ready_dt(leds[i])) {
            return -ENODEV;
        }
        gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);
    }

    set_display(MODE_HAZARD);           /* power-up is the error state */

    k_thread_create(&blinker_thread_data, blinker_stack,
                    K_THREAD_STACK_SIZEOF(blinker_stack),
                    blinker_thread, NULL, NULL, NULL,
                    PRIO_BLINKER, 0, K_NO_WAIT);
    k_thread_name_set(&blinker_thread_data, "blinker");
    return 0;
}