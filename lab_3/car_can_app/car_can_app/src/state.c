/*
 * state.c - zone error state and handling of accepted frames
 *
 * Error state (Lab 1 R3, Lab 3 Part 5):
 *   in error  <=> any reason bit set
 *   power-up:  CTRL_LOST | ZONE_LOST set at boot
 *   out:       heartbeats from both other zones seen (either bus) and a
 *              valid CTRL received (HB_MON clears ZONE_LOST / CTRL_LOST)
 *   in:        test button (TEST), out-of-range CTRL (BADCMD), three
 *              missed heartbeats on both buses (ZONE_LOST), another zone
 *              reporting TEST/BADCMD (REMOTE), CTRL stream stopped
 *
 * app_on_frame() runs in DISPATCH for every first copy of a frame.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <string.h>

#include "state.h"
#include "node.h"
#include "canbus.h"
#include "catalog.h"
#include "config.h"

static const struct gpio_dt_spec tp_err = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_err), gpios);

static atomic_t reasons = ATOMIC_INIT(ERR_CTRL_LOST | ERR_ZONE_LOST);

/* Shared with STEER / DRIVE / HB_MON: spinlock, not volatile */
static struct k_spinlock lock;
static struct ctrl_cmd ctrl;
static struct peer_info peers[4];               /* index = zone id */
static atomic_t last_remote_fault_ms;
static atomic_t last_badcmd_ms;

/* ------------------------------------------------------------------ */
/* Error state                                                        */
/* ------------------------------------------------------------------ */

static atomic_t err_entries;               /* times this zone went NORMAL -> ERROR */
static atomic_t err_cause_count[5];         /* per reason bit: times it newly set */

void state_get_diag(uint32_t *entries, uint32_t counts[5])
{
    *entries = (uint32_t)atomic_get(&err_entries);
    for (int i = 0; i < 5; i++) {
        counts[i] = (uint32_t)atomic_get(&err_cause_count[i]);
    }
}

void state_set(uint32_t bits)
{
    atomic_val_t old = atomic_or(&reasons, (atomic_val_t)bits);
    atomic_val_t newly = (atomic_val_t)bits & ~old;

    for (int i = 0; i < 5; i++) {
        if (newly & (1 << i)) {
            atomic_inc(&err_cause_count[i]);
        }
    }
    if (old == 0 && bits != 0) {
        atomic_inc(&err_entries);
        gpio_pin_set_dt(&tp_err, 1);
        node_apply_fail_safe();                 /* local fail-safe right now */
        blink_kick();                           /* hazards */
    }
}

void state_clear(uint32_t bits)
{
    atomic_val_t old = atomic_and(&reasons, ~(atomic_val_t)bits);

    if (old != 0 && (old & ~(atomic_val_t)bits) == 0) {
        gpio_pin_set_dt(&tp_err, 0);
        blink_kick();
        node_on_ctrl();                         /* actuators resume from the latest CTRL */
    }
}

uint32_t state_reasons(void)
{
    return (uint32_t)atomic_get(&reasons);
}

bool state_in_error(void)
{
    return atomic_get(&reasons) != 0;
}

void state_get_ctrl(struct ctrl_cmd *out)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    *out = ctrl;
    k_spin_unlock(&lock, key);
}

void state_get_peer(uint8_t zone, struct peer_info *out)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    *out = peers[zone & 3];
    k_spin_unlock(&lock, key);
}

uint32_t state_last_remote_fault_ms(void)
{
    return (uint32_t)atomic_get(&last_remote_fault_ms);
}

uint32_t state_last_badcmd_ms(void)
{
    return (uint32_t)atomic_get(&last_badcmd_ms);
}

void state_send_fault(uint8_t failing_zone, uint8_t reason)
{
    uint8_t d[8] = { 0 };
    d[1] = failing_zone;
    d[2] = reason;
    d[3] = NODE_ID;
    canbus_send_both(ID_FAULT, d, false);
}

/* ------------------------------------------------------------------ */
/* Frame handlers (DISPATCH thread)                                    */
/* ------------------------------------------------------------------ */

static bool ctrl_valid(const uint8_t *d)
{
    int16_t steer = (int16_t)(d[1] | (d[2] << 8));

    return cat_crc8(d, 6) == d[6] &&
           (d[5] & CTRL_BTN_RESERVED) == 0 &&
           d[7] == 0 &&
           steer != INT16_MIN;
}

static void on_ctrl(const uint8_t *d, uint32_t now)
{
    if (!ctrl_valid(d)) {
        atomic_set(&last_badcmd_ms, (atomic_val_t)now);
        bool first = (state_reasons() & ERR_BADCMD) == 0;
        state_set(ERR_BADCMD);
        if (first) {
            state_send_fault(NODE_ID, FAULT_BAD_CTRL);
        }
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    ctrl.valid = true;
    ctrl.steer = (int16_t)(d[1] | (d[2] << 8));
    ctrl.thr = d[3];
    ctrl.brk = d[4];
    ctrl.buttons = d[5];
    ctrl.t_ms = now;
    k_spin_unlock(&lock, key);

    node_on_ctrl();                             /* wake STEER / DRIVE */
}

static void on_heartbeat(uint8_t zone, const uint8_t *d, uint32_t now)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    peers[zone].last_hb_ms = now ? now : 1;
    peers[zone].zone_state = d[1];
    peers[zone].bus_health = d[2];
    peers[zone].reasons = d[3];
    k_spin_unlock(&lock, key);

    if (d[3] & ERR_PROPAGATE) {
        atomic_set(&last_remote_fault_ms, (atomic_val_t)now);
        state_set(ERR_REMOTE);
    }
}

static void on_fault(const uint8_t *d, uint32_t now)
{
    if (d[2] == FAULT_TEST_BUTTON || d[2] == FAULT_BAD_CTRL) {
        atomic_set(&last_remote_fault_ms, (atomic_val_t)now);
        state_set(ERR_REMOTE);                  /* fast path: no waiting for heartbeats */
    }
    /* FAULT_ZONE_LOST is informational: every zone runs its own HB_MON. */
}

void app_on_frame(const struct can_frame *f, int bus, uint8_t src)
{
    ARG_UNUSED(bus);
    ARG_UNUSED(src);
    uint32_t now = k_uptime_get_32();

    switch (f->id) {
    case ID_CTRL:
        on_ctrl(f->data, now);
        break;
    case ID_BLINK:
        blink_on_frame(f->data[1], f->data[2]);
        break;
    case ID_HB_N1:
        on_heartbeat(ZONE_N1, f->data, now);
        break;
    case ID_HB_N2:
        on_heartbeat(ZONE_N2, f->data, now);
#if NODE_ID == 3
        blink_follow(&f->data[5]);              /* keep local hazards in step with N2 */
#endif
        break;
    case ID_HB_N3:
        on_heartbeat(ZONE_N3, f->data, now);
        break;
    case ID_FAULT:
        on_fault(f->data, now);
        break;
    default:
        break;
    }
}

void state_init(void)
{
    gpio_pin_configure_dt(&tp_err, GPIO_OUTPUT_ACTIVE);  /* power-up is the error state */
    memset(peers, 0, sizeof(peers));
}
