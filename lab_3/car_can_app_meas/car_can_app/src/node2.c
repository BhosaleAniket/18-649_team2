/*
 * node2.c - N2 front / steering zone: STEER, FORCE_TX (RELAY: relay.c)
 *
 * STEER (prio 3): woken by every valid CTRL; maps steer to a servo angle
 *   and writes the 50 Hz PWM on SRV (PB8). Error state = servo centered.
 * FORCE_TX (prio 9): every 50 ms, FORCE (0x300) with a dummy counter.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "canbus.h"
#include "catalog.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "wcet.h"

static const struct device *const servo = DEVICE_DT_GET(DT_NODELABEL(servo_pwm));

#define SERVO_CHANNEL   1           /* TIM10_CH1 */
#define SERVO_PERIOD_US 20000

K_MUTEX_DEFINE(act_lock);           /* fail-safe vs. normal servo writes */

/* ------------------------------------------------------------------ */
/* STEER                                                              */
/* ------------------------------------------------------------------ */

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void servo_write_deg(float deg)
{
    float lo = SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_LEFT_DEG : SERVO_RIGHT_DEG;
    float hi = SERVO_LEFT_DEG < SERVO_RIGHT_DEG ? SERVO_RIGHT_DEG : SERVO_LEFT_DEG;
    deg = clampf(deg, lo, hi);
    uint32_t us = 500U + (uint32_t)(deg * (2000.0f / 180.0f));   /* 0..180 deg = 500..2500 us */
    pwm_set(servo, SERVO_CHANNEL, PWM_USEC(SERVO_PERIOD_US), PWM_USEC(us), PWM_POLARITY_NORMAL);
}

static float steer_to_deg(int16_t steer)
{
    float x = clampf(steer / (float)STEER_FULL_SCALE, -1.0f, 1.0f);
    if (x >= 0.0f) {
        return SERVO_CENTER_DEG + x * (SERVO_RIGHT_DEG - SERVO_CENTER_DEG);
    }
    return SERVO_CENTER_DEG + (-x) * (SERVO_LEFT_DEG - SERVO_CENTER_DEG);
}

K_SEM_DEFINE(ctrl_sem, 0, 1);

void node_on_ctrl(void)
{
    k_sem_give(&ctrl_sem);
}

void node_apply_fail_safe(void)
{
    k_mutex_lock(&act_lock, K_FOREVER);
    servo_write_deg(SERVO_CENTER_DEG);
    k_mutex_unlock(&act_lock);
}

K_THREAD_STACK_DEFINE(steer_stack, STACK_MEDIUM);
static struct k_thread steer_thread;

static void steer_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_sem_take(&ctrl_sem, K_FOREVER);
        WCET_BEGIN(steer);
        struct ctrl_cmd cmd;
        state_get_ctrl(&cmd);
        float deg = (state_in_error() || !cmd.valid) ? SERVO_CENTER_DEG : steer_to_deg(cmd.steer);

        k_mutex_lock(&act_lock, K_FOREVER);
        WCET_BEGIN(cs);
        servo_write_deg(deg);
        WCET_END(W_ACT_CS, cs);
        k_mutex_unlock(&act_lock);
        WCET_END(W_STEER, steer);
    }
}

/* ------------------------------------------------------------------ */
/* FORCE_TX                                                           */
/* ------------------------------------------------------------------ */

K_TIMER_DEFINE(force_timer, NULL, NULL);
K_THREAD_STACK_DEFINE(force_stack, STACK_SMALL);
static struct k_thread force_thread;

static void force_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    int8_t counter = 0;

    k_timer_start(&force_timer, K_MSEC(FORCE_PERIOD_MS), K_MSEC(FORCE_PERIOD_MS));
    while (1) {
        k_timer_status_sync(&force_timer);
        WCET_BEGIN(force);
        uint8_t d[8] = { 0 };
        d[1] = (uint8_t)counter++;              /* dummy force value */
        canbus_send_both(ID_FORCE, d, false);
        WCET_END(W_FORCE_TX, force);
        wdt_checkin(WDT_BIT_NODE);
    }
}

/* ------------------------------------------------------------------ */
/* Report helper and init                                             */
/* ------------------------------------------------------------------ */

int node_init(void)
{
    if (!device_is_ready(servo)) {
        return -ENODEV;
    }
    servo_write_deg(SERVO_CENTER_DEG);          /* power-up = error = centered */

    k_thread_create(&steer_thread, steer_stack, K_THREAD_STACK_SIZEOF(steer_stack),
                    steer_fn, NULL, NULL, NULL, PRIO_ACTUATE, 0, K_NO_WAIT);
    k_thread_name_set(&steer_thread, "steer");
    k_thread_create(&force_thread, force_stack, K_THREAD_STACK_SIZEOF(force_stack),
                    force_fn, NULL, NULL, NULL, PRIO_FORCE_TX, 0, K_NO_WAIT);
    k_thread_name_set(&force_thread, "force_tx");
    return 0;
}
