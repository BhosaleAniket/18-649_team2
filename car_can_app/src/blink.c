/*
 * blink.c - BLINK_F (N2) / BLINK_R (N3)
 *
 * The cockpit is the blinker master. It sends BLINK (state + phase) on
 * every state change and on every 500 ms toggle edge. This zone drives
 * its two lamps the moment a BLINK frame is handed over, so the front /
 * rear edge difference is the difference of the two receive paths, not
 * of two free-running clocks.
 *
 * In the error state the zone must show hazards even if the cockpit has
 * not (yet) switched BLINK to hazard, or the cockpit is gone: then it
 * falls back to its own 500 ms hazard timer (re-evaluated every 20 ms).
 *
 * Keeping the local timers of N2 and N3 in step (cold start with the
 * proxy stopped, or the cockpit lost):
 *   - every BLINK frame re-anchors the local timer to the cockpit's phase,
 *     so a fallback after the cockpit disappears continues in phase;
 *   - N2 puts "ms to my next edge" and "lamps on now" in its heartbeat
 *     (blink_sync_info) and N3 re-anchors to it on every HB_N2
 *     (blink_follow), so the two local timers stay within the receive
 *     latency of each other even when the cockpit was never there.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>

#include "catalog.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "wcet.h"

#define BLINK_STALE_MS   1200       /* > two 500 ms toggles without BLINK = cockpit gone */
#define LOCAL_HALF_MS    500        /* 1 Hz hazard when running on our own timer */

static const struct gpio_dt_spec lamp_l = GPIO_DT_SPEC_GET(DT_NODELABEL(blink_left), gpios);
static const struct gpio_dt_spec lamp_r = GPIO_DT_SPEC_GET(DT_NODELABEL(blink_right), gpios);

static struct k_spinlock lock;
static uint8_t rx_state = BLINK_OFF;
static uint8_t rx_phase;
static uint32_t rx_ms;
static bool rx_any;
static uint32_t epoch_ms;          /* local hazard: lamps on during [epoch + 2k*500, epoch + (2k+1)*500) */

K_SEM_DEFINE(blink_sem, 0, 1);

void blink_on_frame(uint8_t state, uint8_t phase)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    rx_state = state;
    rx_phase = phase;
    rx_ms = k_uptime_get_32();
    rx_any = true;
    /* re-anchor the local timer: an "on" phase starts now, an "off" phase
     * started one half-period ago */
    epoch_ms = phase ? rx_ms : rx_ms - LOCAL_HALF_MS;
    k_spin_unlock(&lock, key);
    k_sem_give(&blink_sem);
}

static bool local_on(uint32_t now)
{
    return (((now - epoch_ms) / LOCAL_HALF_MS) % 2) == 0;
}

void blink_sync_info(uint8_t out[3])
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t now = k_uptime_get_32();
    uint32_t to_edge = LOCAL_HALF_MS - ((now - epoch_ms) % LOCAL_HALF_MS);
    bool on = local_on(now);
    k_spin_unlock(&lock, key);
    out[0] = (uint8_t)(to_edge & 0xFF);
    out[1] = (uint8_t)(to_edge >> 8);
    out[2] = on ? 1 : 0;
}

void blink_follow(const uint8_t in[3])
{
    uint32_t to_edge = (uint32_t)in[0] | ((uint32_t)in[1] << 8);
    if (to_edge == 0 || to_edge > LOCAL_HALF_MS) {
        return;                                 /* not a sync field (old firmware / cockpit) */
    }
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t now = k_uptime_get_32();
    uint32_t into_half = LOCAL_HALF_MS - to_edge;
    epoch_ms = now - into_half - (in[2] ? 0 : LOCAL_HALF_MS);
    k_spin_unlock(&lock, key);
}

void blink_kick(void)
{
    k_sem_give(&blink_sem);
}

static void lamps(uint8_t state, bool on)
{
    bool l = on && (state == BLINK_LEFT || state == BLINK_HAZARD);
    bool r = on && (state == BLINK_RIGHT || state == BLINK_HAZARD);
    gpio_pin_set_dt(&lamp_l, l);
    gpio_pin_set_dt(&lamp_r, r);
}

K_THREAD_STACK_DEFINE(blink_stack, STACK_SMALL);
static struct k_thread blink_thread;

static void blink_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        k_sem_take(&blink_sem, K_MSEC(BLINK_TICK_MS));
        WCET_BEGIN(blink);

        uint32_t now = k_uptime_get_32();
        k_spinlock_key_t key = k_spin_lock(&lock);
        uint8_t st = rx_state, ph = rx_phase;
        bool fresh = rx_any && (now - rx_ms) < BLINK_STALE_MS;
        bool lon = local_on(now);
        k_spin_unlock(&lock, key);

        bool err = state_in_error();
        if (fresh && (!err || st == BLINK_HAZARD)) {
            lamps(st, ph != 0);                         /* follow the cockpit */
        } else if (err) {
            lamps(BLINK_HAZARD, lon);                   /* local hazards, kept in step (above) */
        } else {
            lamps(BLINK_OFF, false);
        }
        WCET_END(W_BLINK, blink);
    }
}

int blink_init(void)
{
    if (!gpio_is_ready_dt(&lamp_l) || !gpio_is_ready_dt(&lamp_r)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&lamp_l, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&lamp_r, GPIO_OUTPUT_INACTIVE);

    k_thread_create(&blink_thread, blink_stack, K_THREAD_STACK_SIZEOF(blink_stack),
                    blink_fn, NULL, NULL, NULL, PRIO_BLINK, 0, K_NO_WAIT);
    k_thread_name_set(&blink_thread, "blink");
    return 0;
}
