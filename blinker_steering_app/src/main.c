/*
 * steering_blinker_app - steering servo + blinkers with self-cancel (Nucleo-F401RE)
 *
 * Bench test for:
 *   - turn signals that self-cancel when the wheel passes a turn threshold
 *     and returns back past it
 *   - one side cancelling the other
 *   - hazards turning on and off (cancelling back to the turn signal state)
 *
 * The steering value is simulated from the keyboard, in the same units the
 * Pi link will send (steer: -32768 full left .. 0 center .. 32767 full right).
 * The servo follows it, so you can see where the wheel "is".
 *
 * Serial keys (ST-LINK COM port, 115200):
 *   l / r    press the left / right signal button
 *            (pressing the active side again turns it off)
 *   h        toggle hazards (also: user button PC13)
 *   a / d    steer left / right by 5% of full lock
 *   A / D    steer left / right by 25% of full lock
 *   c        steering to center
 *   s        print current state
 *
 * Self-cancel rule, per side (shown for LEFT, RIGHT is mirrored):
 *   1. LEFT is active, not armed.
 *   2. Steering goes past -TURN_THRESHOLD         -> armed.
 *   3. Steering comes back above
 *      -(TURN_THRESHOLD - TURN_HYSTERESIS)        -> LEFT cancels.
 *   The hysteresis stops the signal from cancelling on noise when the
 *   wheel hovers right at the threshold. Pressing either side clears
 *   the armed flag, so a new signal must see its own turn before it
 *   can self-cancel.
 *
 * Hazards override the display but do not erase the turn state:
 * cancelling hazards returns to whatever turn signal was active.
 *
 * Blink timing is the same as blinker_app: a periodic 250 ms k_timer,
 * 1 Hz for turn signals, 2 Hz for hazards, 50% duty.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
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

static const struct device *servo_pwm = DEVICE_DT_GET(DT_NODELABEL(pwm10));
static const struct device *gpioc_dev = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *console   = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define SERVO_CHANNEL  1       /* TIM10_CH1 */
#define BUTTON_PIN     13      /* PC13, active low */

/* ------------------------------------------------------------------ */
/* Steering (wheel units)                                             */
/* ------------------------------------------------------------------ */

#define STEER_MIN          (-32768)
#define STEER_MAX          32767
#define STEER_STEP_FINE    1638    /* ~5% of full lock  */
#define STEER_STEP_COARSE  8192    /* 25% of full lock  */

#define TURN_THRESHOLD     8192    /* 25% of full lock: counts as "turning" */
#define TURN_HYSTERESIS    1638    /* must come back 5% past it to cancel   */

/* ------------------------------------------------------------------ */
/* Servo (paste your calibrated values from servo_app)                */
/* ------------------------------------------------------------------ */

#define SERVO_PERIOD_US    20000
#define SERVO_MIN_US       500
#define SERVO_MAX_US       2500
#define SERVO_RANGE_DEG    180.0f

#define SERVO_LEFT_DEG     72.0f    /* placeholder */
#define SERVO_CENTER_DEG   90.0f    /* placeholder */
#define SERVO_RIGHT_DEG    108.0f   /* placeholder */

#define SERVO_LO_DEG (SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_LEFT_DEG : SERVO_RIGHT_DEG)
#define SERVO_HI_DEG (SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_RIGHT_DEG : SERVO_LEFT_DEG)

/* ------------------------------------------------------------------ */
/* Timing                                                             */
/* ------------------------------------------------------------------ */

#define TICK_MS   250      /* blink timer: half-period of the 2 Hz hazard */
#define LOOP_MS   20       /* input + state machine loop */

/* ------------------------------------------------------------------ */
/* Servo                                                              */
/* ------------------------------------------------------------------ */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float steer_to_servo_deg(int32_t steer)
{
    if (steer >= 0) {
        return SERVO_CENTER_DEG +
               (steer / 32767.0f) * (SERVO_RIGHT_DEG - SERVO_CENTER_DEG);
    }
    return SERVO_CENTER_DEG +
           (steer / 32768.0f) * (SERVO_CENTER_DEG - SERVO_LEFT_DEG);
}

static void servo_write_deg(float deg)
{
    deg = clampf(deg, SERVO_LO_DEG, SERVO_HI_DEG);
    float us = SERVO_MIN_US + deg * ((SERVO_MAX_US - SERVO_MIN_US) / SERVO_RANGE_DEG);
    pwm_set(servo_pwm, SERVO_CHANNEL,
            PWM_USEC(SERVO_PERIOD_US), PWM_USEC((uint32_t)(us + 0.5f)), 0);
}

/* ------------------------------------------------------------------ */
/* Blink output (same design as blinker_app)                          */
/* ------------------------------------------------------------------ */

enum blink_mode { MODE_OFF, MODE_LEFT, MODE_RIGHT, MODE_HAZARD, NUM_MODES };

static const char *const mode_names[NUM_MODES] = {
    "OFF", "LEFT", "RIGHT", "HAZARD"
};

static atomic_t display_mode = ATOMIC_INIT(MODE_OFF);  /* thread -> ISR */
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

/* Interrupt context: read mode, write pins, nothing else. */
static void blink_tick(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    enum blink_mode m = (enum blink_mode)atomic_get(&display_mode);
    ticks++;

    bool on = (m == MODE_HAZARD) ? ((ticks % 2) == 0)
                                 : (((ticks / 2) % 2) == 0);
    apply_outputs(m, on);
}

K_TIMER_DEFINE(blink_timer, blink_tick, NULL);

/* Change the displayed pattern. Only restarts the blink if it changed,
 * so an unchanged mode keeps its phase instead of restarting. */
static void set_display(enum blink_mode m)
{
    if ((enum blink_mode)atomic_get(&display_mode) == m) {
        return;
    }

    k_timer_stop(&blink_timer);
    atomic_set(&display_mode, m);
    ticks = 0;

    if (m == MODE_OFF) {
        apply_outputs(m, false);
    } else {
        apply_outputs(m, true);
        k_timer_start(&blink_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    }

    printk("[%u ms] display -> %s\n", (uint32_t)k_uptime_get(), mode_names[m]);
}

/* ------------------------------------------------------------------ */
/* Turn signal state machine (thread context only)                    */
/* ------------------------------------------------------------------ */

enum turn_state { TURN_OFF, TURN_LEFT, TURN_RIGHT };

static const char *const turn_names[] = { "OFF", "LEFT", "RIGHT" };

static enum turn_state turn = TURN_OFF;
static bool turn_armed = false;     /* has the wheel passed the threshold yet? */
static bool hazard = false;
static int32_t steer = 0;

static void log_event(const char *what)
{
    printk("[%u ms] %s (turn=%s armed=%d hazard=%d steer=%d)\n",
           (uint32_t)k_uptime_get(), what, turn_names[turn],
           turn_armed, hazard, (int)steer);
}

static void press_left(void)
{
    turn = (turn == TURN_LEFT) ? TURN_OFF : TURN_LEFT;
    turn_armed = false;
    log_event("left pressed");
}

static void press_right(void)
{
    turn = (turn == TURN_RIGHT) ? TURN_OFF : TURN_RIGHT;
    turn_armed = false;
    log_event("right pressed");
}

static void toggle_hazard(void)
{
    hazard = !hazard;
    log_event(hazard ? "hazards on" : "hazards cancelled");
}

static void update_self_cancel(void)
{
    if (turn == TURN_LEFT) {
        if (!turn_armed && steer <= -TURN_THRESHOLD) {
            turn_armed = true;
            log_event("left armed: turned past threshold");
        } else if (turn_armed && steer > -(TURN_THRESHOLD - TURN_HYSTERESIS)) {
            turn = TURN_OFF;
            turn_armed = false;
            log_event("left self-cancelled: returned past threshold");
        }
    } else if (turn == TURN_RIGHT) {
        if (!turn_armed && steer >= TURN_THRESHOLD) {
            turn_armed = true;
            log_event("right armed: turned past threshold");
        } else if (turn_armed && steer < (TURN_THRESHOLD - TURN_HYSTERESIS)) {
            turn = TURN_OFF;
            turn_armed = false;
            log_event("right self-cancelled: returned past threshold");
        }
    }
}

static enum blink_mode desired_display(void)
{
    if (hazard) {
        return MODE_HAZARD;             /* hazards override turn signals */
    }
    switch (turn) {
    case TURN_LEFT:  return MODE_LEFT;
    case TURN_RIGHT: return MODE_RIGHT;
    default:         return MODE_OFF;
    }
}

static void steer_by(int32_t delta)
{
    steer += delta;
    if (steer < STEER_MIN) { steer = STEER_MIN; }
    if (steer > STEER_MAX) { steer = STEER_MAX; }
    printk("steer = %d (%d%% of full lock)\n", (int)steer,
           (int)((steer * 100) / (steer >= 0 ? STEER_MAX : -STEER_MIN)));
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

static bool led_ready(const struct gpio_dt_spec *led)
{
    if (!gpio_is_ready_dt(led)) {
        return false;
    }
    gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
    return true;
}

int main(void)
{
    if (!led_ready(&led_fl) || !led_ready(&led_fr) ||
        !led_ready(&led_rl) || !led_ready(&led_rr) ||
        !device_is_ready(servo_pwm) || !device_is_ready(gpioc_dev) ||
        !device_is_ready(console)) {
        printk("ERROR: device not ready\n");
        return 0;
    }

    gpio_pin_configure(gpioc_dev, BUTTON_PIN, GPIO_INPUT);
    servo_write_deg(steer_to_servo_deg(steer));

    printk("steering_blinker_app ready\n");
    printk("keys: l r signals | h hazard | a/d A/D steer | c center | s status\n");
    printk("turn threshold = %d (%d%% of full lock), hysteresis = %d\n",
           TURN_THRESHOLD, (TURN_THRESHOLD * 100) / STEER_MAX, TURN_HYSTERESIS);

    bool prev_pressed = false;

    while (1) {
        /* Keys */
        unsigned char c;
        if (uart_poll_in(console, &c) == 0) {
            switch (c) {
            case 'l': press_left();                  break;
            case 'r': press_right();                 break;
            case 'h': toggle_hazard();               break;
            case 'a': steer_by(-STEER_STEP_FINE);    break;
            case 'd': steer_by(+STEER_STEP_FINE);    break;
            case 'A': steer_by(-STEER_STEP_COARSE);  break;
            case 'D': steer_by(+STEER_STEP_COARSE);  break;
            case 'c': steer = 0; printk("steer = 0 (center)\n"); break;
            case 's': log_event("status");           break;
            default: break;
            }
        }

        /* Button: toggle hazards (stands in for the self-test button) */
        bool pressed = (gpio_pin_get(gpioc_dev, BUTTON_PIN) == 0);
        if (pressed && !prev_pressed) {
            toggle_hazard();
        }
        prev_pressed = pressed;

        /* State machine, then outputs */
        update_self_cancel();
        set_display(desired_display());
        servo_write_deg(steer_to_servo_deg(steer));

        k_msleep(LOOP_MS);
    }

    return 0;
}