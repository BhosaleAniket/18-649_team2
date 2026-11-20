/*
 * blink.c - BLINK_F (N2) / BLINK_R (N3), Lab 5: lamp edges on global time
 *
 * What to show:
 *   - in the error state: hazards
 *   - otherwise: the cockpit's BLINK state (left / right / hazard / off),
 *     as long as BLINK frames keep coming (BLINK_STALE_MS)
 * When to switch: every node derives the edges from global_gettime(), so
 * the front and rear lamps switch at the same global instant whatever the
 * receive path of the BLINK frame was. Lamps are on in the first half of
 * each period: on while (global / half) is even.
 *   - normal blinking and hazards: BLINK_HALF_US_NORMAL (2 Hz)
 *   - failure mode (cockpit lost: no HB_N1 on either bus for 66 ms):
 *     BLINK_HALF_US_FAILURE (4 Hz)
 * The phase byte in BLINK is no longer used.
 *
 * Precision: the thread sleeps until EDGE_WAKE_EARLY_US before the edge,
 * then spins to the exact microsecond. It is cooperative (PRIO_EDGE), so
 * nothing preempts it in that short window. Every wakeup recomputes the
 * edge from the current clock model, sleeps are capped at one half-period,
 * and the spin is capped at 2 x EDGE_WAKE_EARLY_US: a clock step (first
 * sync can move global time by hours) can never leave it spinning.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>

#include "catalog.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "raft_node.h"
#include "wcet.h"

#define BLINK_STALE_MS   1200       /* > two toggles without BLINK = cockpit gone */

static const struct gpio_dt_spec lamp_l = GPIO_DT_SPEC_GET(DT_NODELABEL(blink_left), gpios);
static const struct gpio_dt_spec lamp_r = GPIO_DT_SPEC_GET(DT_NODELABEL(blink_right), gpios);

static struct k_spinlock lock;
static uint8_t rx_state = BLINK_OFF;
static uint32_t rx_ms;
static bool rx_any;

K_SEM_DEFINE(blink_sem, 0, 1);

void blink_on_frame(uint8_t state, uint8_t phase)
{
    ARG_UNUSED(phase);
    k_spinlock_key_t key = k_spin_lock(&lock);
    rx_state = state;
    rx_ms = k_uptime_get_32();
    rx_any = true;
    k_spin_unlock(&lock, key);
    k_sem_give(&blink_sem);
}

/* Lab 4 hazard alignment through HB_N2 is replaced by global time */
void blink_sync_info(uint8_t out[3])
{
    out[0] = out[1] = out[2] = 0;
}

void blink_follow(const uint8_t in[3])
{
    ARG_UNUSED(in);
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

/* Failure mode is local and fast: HB_N1 missing on both buses for 66 ms.
 * The committed COCKPIT entry only records it (and also flips when the
 * cockpit is alive but in error, e.g. proxy stopped), so it is not used here. */
static bool cockpit_lost(uint32_t now_ms)
{
    struct peer_info p;
    state_get_peer(ZONE_N1, &p);
    return !(p.last_hb_ms != 0 && (int32_t)(now_ms - p.last_hb_ms) < (int32_t)ZONE_TIMEOUT_MS);
}

static void what_to_show(uint8_t *state, uint32_t *half_us)
{
    uint32_t now = k_uptime_get_32();
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint8_t st = rx_state;
    bool fresh = rx_any && (now - rx_ms) < BLINK_STALE_MS;
    k_spin_unlock(&lock, key);

    bool lost = cockpit_lost(now);
    *half_us = lost ? BLINK_HALF_US_FAILURE : BLINK_HALF_US_NORMAL;
    if (state_in_error()) {
        *state = BLINK_HAZARD;
    } else if (fresh) {
        *state = st;
    } else {
        *state = BLINK_OFF;
    }
}

static bool phase_on(uint64_t global_us, uint32_t half_us)
{
    return ((global_us / half_us) & 1u) == 0;
}

K_THREAD_STACK_DEFINE(blink_stack, STACK_SMALL);
static struct k_thread blink_thread;

static void blink_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        uint8_t st;
        uint32_t half;
        what_to_show(&st, &half);

        /* Every pass recomputes the next edge from the clock model as it is
         * NOW. A REF can step the clock while we sleep (first sync: by
         * hours); an edge computed before the step is never reused. */
        uint64_t now = raft_node_local_us();
        uint64_t g = raft_node_global_us(now);
        lamps(st, phase_on(g, half));           /* current phase (a state change applies at once) */
        uint64_t edge_g = (g / half + 1u) * half;
        uint64_t edge_l = raft_node_local_for_global(edge_g);

        if (edge_l <= now || edge_l - now > EDGE_WAKE_EARLY_US) {
            /* Not close yet (or the model moved): sleep, capped, then recompute */
            uint64_t wait = (edge_l > now + EDGE_WAKE_EARLY_US) ? edge_l - now - EDGE_WAKE_EARLY_US : 100u;
            if (wait > half) {
                wait = half;
            }
            k_sem_take(&blink_sem, K_USEC((uint32_t)wait));   /* BLINK or error change wakes us early */
            continue;
        }

        /* Within EDGE_WAKE_EARLY_US of a freshly computed edge. This thread
         * is cooperative, so the model cannot change during the spin; the
         * spin is capped anyway. */
        uint64_t spin_end = now + 2u * EDGE_WAKE_EARLY_US;
        uint64_t t;
        while ((t = raft_node_local_us()) < edge_l && t < spin_end) {
        }
        WCET_BEGIN(blink);
        what_to_show(&st, &half);
        lamps(st, phase_on(edge_g, half));
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
                    blink_fn, NULL, NULL, NULL, PRIO_EDGE, 0, K_NO_WAIT);
    k_thread_name_set(&blink_thread, "blink");
    return 0;
}
