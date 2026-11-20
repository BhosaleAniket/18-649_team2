/*
 * canbus.c - two MCP2515 controllers, one dispatch path
 *
 * Receive path (per frame, per bus):
 *   INT pin falls -> driver GPIO ISR gives a semaphore
 *   -> MCP2515 interrupt thread (cooperative, above all our threads)
 *      reads the frame over SPI and calls rx_cb()
 *   -> rx_cb: stamp the bus as alive, put the frame on rx_q. No printing,
 *      no blocking.
 *   -> DISPATCH thread: toggle CAN_RX_x ("when dispatch first gets the
 *      frame"), drop our own / duplicate / stale copies by
 *      (ID, sender, seq), hand first copies to RELAY and to
 *      app_on_frame(). Nothing above DISPATCH sees a duplicate.
 *
 * Transmit: canbus_send_both() bumps the per-ID sequence counter once and
 * queues the frame on A then B without blocking (K_NO_WAIT plus a
 * completion callback), so a dead bus can never stall a task.
 *
 * Lab 5:
 *   - rx_cb stamps every frame with the local microsecond clock first, so
 *     REF timing is taken as close to the frame as this driver allows.
 *   - Raft frames (catalog v0.8, IDs base + sender) skip the Lab 3 seq
 *     filter: they are de-duplicated on (ID, seq, bytes 1-7) with the flags
 *     nibble masked, so a relayed copy matches its original and a catch-up
 *     burst never collides when the 4-bit counter wraps.
 *   - Relaying is decided per frame by the shared route rule
 *     (raft_node_relay_bus), for Raft and Lab 3 frames alike. Raft frames
 *     get the RELAYED flag; Lab 3 frames are forwarded bit for bit (CTRL's
 *     CRC covers byte 0).
 *   - canbus_send_raft() sends REF with a TX-complete callback that stamps
 *     the moment the frame left the controller (tsync).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <stm32_ll_exti.h>

#include <string.h>

#include "canbus.h"
#include "catalog.h"
#include "config.h"
#include "wcet.h"
#include "raft_node.h"
#include "raft/raft_proto.h"

static const struct device *const can_devs[NUM_BUSES] = {
    DEVICE_DT_GET(DT_NODELABEL(can_a)),
    DEVICE_DT_GET(DT_NODELABEL(can_b)),
};

/* MCP2515 interrupt lines (same pins the driver uses) */
static const struct gpio_dt_spec can_int[NUM_BUSES] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(can_a), int_gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(can_b), int_gpios),
};

static const struct gpio_dt_spec tp_rx[NUM_BUSES] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_rx_a), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_rx_b), gpios),
};
static const struct gpio_dt_spec tp_tx[NUM_BUSES] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_tx_a), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_tx_b), gpios),
};

/* IDs this zone listens to (everything in the catalog it consumes),
 * then the Raft ranges (base + sender 1..3) */
struct rx_filter_def {
    uint16_t id;
    uint16_t mask;
};
static const struct rx_filter_def rx_filters[] = {
    { ID_CTRL, CAN_STD_ID_MASK },
    { ID_FAULT, CAN_STD_ID_MASK },
    { ID_HB_N1, CAN_STD_ID_MASK },
#if NODE_ID == 2
    { ID_HB_N3, CAN_STD_ID_MASK },
#else
    { ID_HB_N2, CAN_STD_ID_MASK },
#endif
    { ID_BLINK, CAN_STD_ID_MASK },
    { RAFT_ID_REF, 0x7F8 },
    { RAFT_ID_REQ_VOTE, 0x7F0 },
    { RAFT_ID_VOTE, 0x7F0 },
    { RAFT_ID_APPEND, 0x7F0 },
    { RAFT_ID_ACK, 0x7F0 },
    { RAFT_ID_REPORT, 0x7F0 },
    { RAFT_ID_REDIRECT, 0x7F0 },
};

/* ------------------------------------------------------------------ */
/* Statistics and bus health (atomics: written from several contexts)  */
/* ------------------------------------------------------------------ */

static atomic_t last_rx_ms[NUM_BUSES];
static atomic_t degraded[NUM_BUSES];
static atomic_t st_rx[NUM_BUSES], st_tx_ok[NUM_BUSES], st_tx_err[NUM_BUSES], st_tx_refused[NUM_BUSES];
static atomic_t st_dup, st_stale, st_overflow;

/* ------------------------------------------------------------------ */
/* Receive: driver callback -> queue                                  */
/* ------------------------------------------------------------------ */

struct rx_item {
    struct can_frame frame;
    uint64_t t_us;
    uint32_t t_ms;
    uint8_t bus;
};

K_MSGQ_DEFINE(rx_q, sizeof(struct rx_item), 32, 8);

/* Runs in the MCP2515 interrupt thread (cooperative), not in an ISR. */
static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    uint64_t t_us = raft_node_local_us();      /* first: REF timing */
    uint8_t bus = (uint8_t)(uintptr_t)user_data;
    struct rx_item it = {
        .frame = *frame,
        .t_us = t_us,
        .t_ms = k_uptime_get_32(),
        .bus = bus,
    };

    wcet_rx_delivered(bus);
    atomic_set(&last_rx_ms[bus], (atomic_val_t)it.t_ms);
    atomic_inc(&st_rx[bus]);

    if (k_msgq_put(&rx_q, &it, K_NO_WAIT) != 0) {
        atomic_inc(&st_overflow);
    }
}

/* ------------------------------------------------------------------ */
/* Duplicate / stale filter - DISPATCH thread only, so no lock needed  */
/* ------------------------------------------------------------------ */

struct seen {
    uint16_t id;
    uint8_t src;
    uint8_t seq;
    uint32_t t_ms;
    bool used;
};

static struct seen seen_tab[12];

enum verdict { ACCEPT, DUPLICATE, STALE };

static enum verdict check_seq(uint16_t id, uint8_t src, uint8_t seq, uint32_t now)
{
    struct seen *e = NULL;

    for (int i = 0; i < ARRAY_SIZE(seen_tab); i++) {
        if (seen_tab[i].used && seen_tab[i].id == id && seen_tab[i].src == src) {
            e = &seen_tab[i];
            break;
        }
    }
    if (e == NULL) {
        for (int i = 0; i < ARRAY_SIZE(seen_tab); i++) {
            if (!seen_tab[i].used) {
                e = &seen_tab[i];
                e->used = true;
                e->id = id;
                e->src = src;
                e->seq = seq;
                e->t_ms = now;
                return ACCEPT;
            }
        }
        return ACCEPT;                          /* table full: never drop */
    }

    uint8_t ahead = (uint8_t)((seq - e->seq) & 0x0F);
    uint32_t age = now - e->t_ms;

    if (ahead == 0 && age < DEDUP_WINDOW_MS) {
        return DUPLICATE;                       /* other bus's copy, or the relayed copy */
    }
    if (ahead >= 9 && age < STALE_WINDOW_MS) {
        return STALE;                           /* 1..7 behind: an old buffered frame */
    }
    e->seq = seq;
    e->t_ms = now;
    return ACCEPT;
}

/* Raft frames: duplicate = same ID, seq and bytes 1..7 within the window
 * (flags nibble masked, so a relayed copy matches its original) */
struct raft_seen {
    uint16_t id;
    uint8_t key[8];
    uint32_t t_ms;
};
static struct raft_seen raft_seen_tab[24];
static uint8_t raft_seen_head;

static bool raft_duplicate(const struct can_frame *f, uint32_t now)
{
    uint8_t key[8];
    memcpy(key, f->data, 8);
    key[0] &= 0x0F;
    for (int i = 0; i < ARRAY_SIZE(raft_seen_tab); i++) {
        struct raft_seen *e = &raft_seen_tab[i];
        if (e->t_ms != 0 && e->id == f->id && (now - e->t_ms) < DEDUP_WINDOW_MS &&
            memcmp(e->key, key, 8) == 0) {
            return true;
        }
    }
    struct raft_seen *e = &raft_seen_tab[raft_seen_head];
    raft_seen_head = (uint8_t)((raft_seen_head + 1) % ARRAY_SIZE(raft_seen_tab));
    e->id = (uint16_t)f->id;
    memcpy(e->key, key, 8);
    e->t_ms = now ? now : 1;
    return false;
}

/* ------------------------------------------------------------------ */
/* DISPATCH thread                                                    */
/* ------------------------------------------------------------------ */

K_THREAD_STACK_DEFINE(dispatch_stack, STACK_MEDIUM);
static struct k_thread dispatch_thread;

static void dispatch_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    struct rx_item it;

    while (1) {
        k_msgq_get(&rx_q, &it, K_FOREVER);
        gpio_pin_toggle_dt(&tp_rx[it.bus]);     /* CAN_RX_x: dispatch first gets the frame */
        WCET_BEGIN(disp);

        const struct can_frame *f = &it.frame;
        if (f->dlc != CAT_DLC || (f->flags & CAN_FRAME_RTR)) {
            continue;                           /* not a catalog frame */
        }

        if (raft_id_base(f->id) != 0) {         /* Lab 5 Raft frame */
            uint8_t sender = raft_id_sender(f->id);
            if (sender == NODE_ID) {
                continue;
            }
            if (raft_duplicate(f, it.t_ms)) {
                atomic_inc(&st_dup);
                if (raft_id_base(f->id) == RAFT_ID_REF) {
                    /* Raft ignores it, but time sync needs every copy:
                     * only the copy on the stamped bus can be paired */
                    raft_node_post_frame((uint8_t)(it.bus + 1), f, it.t_us, false);
                }
                continue;
            }
            bool relayed = (f->data[0] & RAFT_FLAG_RELAYED) != 0;
            uint8_t out = raft_node_relay_bus(sender, (uint8_t)(it.bus + 1), relayed);
            if (out != 0) {
                struct can_frame g = *f;
                g.data[0] |= RAFT_FLAG_RELAYED;
                relay_post_to(&g, out - 1);
            }
            raft_node_post_frame((uint8_t)(it.bus + 1), f, it.t_us, true);
            WCET_END(W_DISPATCH, disp);
            continue;
        }

        uint8_t src = cat_source(f->id, f->data);
        if (src == 0 || src == NODE_ID) {
            continue;
        }

        switch (check_seq(f->id, src, f->data[0] & 0x0F, it.t_ms)) {
        case DUPLICATE:
            atomic_inc(&st_dup);
            continue;
        case STALE:
            atomic_inc(&st_stale);
            continue;
        case ACCEPT:
            break;
        }

        uint8_t out = raft_node_relay_bus(src, (uint8_t)(it.bus + 1), false);
        if (out != 0) {
            relay_post_to(f, out - 1);          /* bit for bit: CTRL's CRC covers byte 0 */
        }
        app_on_frame(f, it.bus, src);
        WCET_END(W_DISPATCH, disp);
    }
}

/* ------------------------------------------------------------------ */
/* Transmit                                                           */
/* ------------------------------------------------------------------ */

#define TX_BUFFER_WAIT  K_USEC(600)

/* One counter per ID this zone sends (HB, FAULT, FORCE). */
static atomic_t seq_hb, seq_fault, seq_force, seq_other;

static atomic_t *seq_for(uint16_t id)
{
    switch (id) {
    case ID_HB_N2:
    case ID_HB_N3: return &seq_hb;
    case ID_FAULT: return &seq_fault;
    case ID_FORCE: return &seq_force;
    default:       return &seq_other;
    }
}

/* Runs in the driver's interrupt thread when a frame finishes. */
static void tx_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    int bus = (int)(uintptr_t)user_data;
    wcet_tx_delivered(bus);
    if (error == 0) {
        atomic_inc(&st_tx_ok[bus]);
    } else {
        atomic_inc(&st_tx_err[bus]);
    }
}

/* REF finished on the wire: stamp it for time sync */
static void tx_cb_ref(const struct device *dev, int error, void *user_data)
{
    uint64_t t_us = raft_node_local_us();       /* first */
    int bus = (int)(uintptr_t)user_data;
    tx_cb(dev, error, user_data);
    if (error == 0) {
        raft_node_post_ref_txdone((uint8_t)(bus + 1), t_us);
    }
}

static int queue_on_cb(int bus, const struct can_frame *f, can_tx_callback_t cb)
{
    gpio_pin_toggle_dt(&tp_tx[bus]);            /* CAN_TX_x: the call that queues the frame */
    WCET_BEGIN(spi);
    /* Wait up to TX_BUFFER_WAIT for one of the 3 MCP2515 TX buffers to free
     * up (one frame on the wire is 270 us), instead of dropping the frame
     * when two or three others are still queued. The completion is still a
     * callback, so this never waits for the frame itself to be sent. */
    int err = can_send(can_devs[bus], f, TX_BUFFER_WAIT, cb, (void *)(uintptr_t)bus);
    WCET_END(W_SPI_TX, spi);
    if (err != 0) {
        atomic_inc(&st_tx_refused[bus]);        /* buffers full (no ACK) or controller stopped */
    }
    return err;
}

static int queue_on(int bus, const struct can_frame *f)
{
    return queue_on_cb(bus, f, tx_cb);
}

void canbus_send_both(uint16_t id, uint8_t data[8], bool is_heartbeat)
{
    uint8_t seq = (uint8_t)atomic_inc(seq_for(id));    /* bumped once for both copies */

    struct can_frame f = {
        .id = id,
        .dlc = CAT_DLC,
        .flags = 0,
    };
    data[0] = (uint8_t)((data[0] & 0xF0) | (seq & 0x0F));
    memcpy(f.data, data, CAT_DLC);

    for (int bus = 0; bus < NUM_BUSES; bus++) {
        if (atomic_get(&degraded[bus]) && !is_heartbeat) {
            continue;
        }
        queue_on(bus, &f);
    }
}

/* One counter per Raft frame type (all sent by this node only) */
static atomic_t seq_raft[8];

int canbus_send_raft(const raft_frame_t *rf, bool is_ref)
{
    uint16_t base = raft_id_base(rf->id);
    int k = (base == RAFT_ID_REF) ? 0 : (int)((base >> 4) & 0x07);
    uint8_t seq = (uint8_t)atomic_inc(&seq_raft[k]);
    struct can_frame f = {
        .id = rf->id,
        .dlc = CAT_DLC,
        .flags = 0,
    };
    memcpy(f.data, rf->data, CAT_DLC);
    f.data[0] = (uint8_t)((f.data[0] & 0xF0) | (seq & 0x0F));

    int sent = 0;
    for (int bus = 0; bus < NUM_BUSES; bus++) {
        if (!(rf->bus_mask & (1u << bus)) || atomic_get(&degraded[bus])) {
            continue;
        }
        if (queue_on_cb(bus, &f, is_ref ? tx_cb_ref : tx_cb) == 0) {
            sent++;
        }
    }
    return sent > 0 ? 0 : -ENETDOWN;
}

int canbus_send_one(int bus, const struct can_frame *frame)
{
    if (atomic_get(&degraded[bus])) {
        return -ENETDOWN;
    }
    return queue_on(bus, frame);
}

/* ------------------------------------------------------------------ */
/* Health                                                             */
/* ------------------------------------------------------------------ */

uint32_t canbus_last_rx_ms(int bus)
{
    return (uint32_t)atomic_get(&last_rx_ms[bus]);
}

bool canbus_is_degraded(int bus)
{
    return atomic_get(&degraded[bus]) != 0;
}

/* Controller recovery. can_start() asks the MCP2515 back into normal mode
 * and gives up after about 1 ms (driver: MCP2515_MODE_CHANGE_TIMEOUT_USEC).
 * If it gives up, the controller stays stopped in configuration mode: it
 * cannot receive, every send returns -ENETDOWN, and DEG (which clears only
 * when a frame arrives) never clears. So the result is checked, and
 * canbus_recover() keeps trying while the bus is degraded. */
static atomic_t st_restarts[NUM_BUSES], st_restart_fail[NUM_BUSES];
static int last_start_err[NUM_BUSES];

static void note_start(int bus, int err, const char *what)
{
    if (err == -EALREADY) {
        err = 0;                                /* already running */
    }
    if (err == 0) {
        atomic_inc(&st_restarts[bus]);
    } else {
        atomic_inc(&st_restart_fail[bus]);
    }
    if (err != last_start_err[bus]) {           /* print changes only */
        printk("CAN %c: %s -> %d%s\n", 'A' + bus, what, err,
               err ? " (controller stopped; retrying every HB_MON tick)" : "");
        last_start_err[bus] = err;
    }
}

void canbus_set_degraded(int bus, bool deg)
{
    bool was = atomic_set(&degraded[bus], deg ? 1 : 0) != 0;

    if (deg && !was) {
        /* Throw away frames stuck in the TX buffers waiting for an ACK,
         * so stale CTRL/HB copies are not sent when the wire comes back.
         * Filters survive stop/start. */
        (void)can_stop(can_devs[bus]);
        note_start(bus, can_start(can_devs[bus]), "restart on DEG");
    }
}

/* Called by HB_MON every tick while the bus is degraded.
 * STOPPED (a failed can_start): start it again, backing off from 100 ms to 1 s.
 * BUS_OFF is left alone: that is CAN fault confinement, and the MCP2515
 * rejoins by itself after 128 x 11 recessive bits. Restarting it put a
 * node with a bad connection straight back on the bus, where its error
 * frames disturbed everyone else's traffic. */
static uint32_t next_start_ms[NUM_BUSES];
static uint32_t start_backoff_ms[NUM_BUSES];

void canbus_recover(int bus)
{
    enum can_state st;
    uint32_t now = k_uptime_get_32();

    if (can_get_state(can_devs[bus], &st, NULL) != 0 || st != CAN_STATE_STOPPED) {
        start_backoff_ms[bus] = 0;
        return;
    }
    if ((int32_t)(now - next_start_ms[bus]) < 0) {
        return;
    }
    int err = can_start(can_devs[bus]);
    note_start(bus, err, "start (was stopped)");
    start_backoff_ms[bus] = err ? MIN(start_backoff_ms[bus] ? start_backoff_ms[bus] * 2 : 100u, 1000u) : 0;
    next_start_ms[bus] = now + start_backoff_ms[bus];
}

/* TX stuck watch, every HB_MON tick on a bus that is NOT degraded (it is
 * receiving fine). If no transmission has completed for TX_STUCK_MS while
 * sends keep being refused, the driver believes all three TX slots are
 * still busy (completions lost, e.g. during an interrupt stall) and will
 * refuse every frame forever. Seen on the bench: tx ok frozen at 210,
 * refused +50/s, DEG off, tec 0. can_stop() completes every slot with
 * -ENETDOWN, which frees them; can_start() resumes. At most once per s. */
#define TX_STUCK_MS 200
static uint32_t txok_seen[NUM_BUSES], refused_seen[NUM_BUSES], txok_change_ms[NUM_BUSES], tx_fix_ms[NUM_BUSES];
static atomic_t st_tx_stuck[NUM_BUSES];

void canbus_check_tx(int bus)
{
    uint32_t now = k_uptime_get_32();
    uint32_t ok = (uint32_t)atomic_get(&st_tx_ok[bus]);
    uint32_t refused = (uint32_t)atomic_get(&st_tx_refused[bus]);

    if (ok != txok_seen[bus] || atomic_get(&degraded[bus])) {
        txok_seen[bus] = ok;
        refused_seen[bus] = refused;
        txok_change_ms[bus] = now;
        return;
    }
    if (refused - refused_seen[bus] < 3 || (now - txok_change_ms[bus]) < TX_STUCK_MS ||
        (now - tx_fix_ms[bus]) < 1000) {
        return;
    }
    /* Only when the controller says all is well. An error-passive or bus-off
     * controller is failing its sends for a real reason, and a restart would
     * reset its error counters, the same mistake as restarting a bus-off one. */
    enum can_state st;
    if (can_get_state(can_devs[bus], &st, NULL) != 0 || st != CAN_STATE_ERROR_ACTIVE) {
        return;
    }
    tx_fix_ms[bus] = now;
    txok_change_ms[bus] = now;
    refused_seen[bus] = refused;
    atomic_val_t n = atomic_inc(&st_tx_stuck[bus]) + 1;
    (void)can_stop(can_devs[bus]);
    int err = can_start(can_devs[bus]);
    note_start(bus, err, "restart (TX stuck)");
    if (n <= 5 || (n % 20) == 0) {
        printk("CAN %c: TX stuck (no completion for %u ms, sends refused), controller restarted (%d so far)\n",
               'A' + bus, TX_STUCK_MS, (int)n);
    }
}

/* Interrupt stall watch, every HB_MON tick, both buses.
 * The MCP2515 holds INT asserted until the driver reads and clears its
 * flags, which takes microseconds. The driver only runs on an INT edge; if
 * an edge is lost, INT stays asserted, nothing is received or completed
 * again, yet the controller reports error-active with zero error counters
 * (seen on the bench: rx and tx-ok frozen, every send refused). Asserted on
 * two ticks in a row = stalled: raise that pin's EXTI line in software,
 * which runs the driver's interrupt handler exactly as a real edge would. */
static uint8_t int_asserted_ticks[NUM_BUSES];
static atomic_t st_int_kicks[NUM_BUSES];

void canbus_check_int(int bus)
{
    if (gpio_pin_get_dt(&can_int[bus]) <= 0) {
        int_asserted_ticks[bus] = 0;
        return;
    }
    if (++int_asserted_ticks[bus] < 2) {
        return;
    }
    int_asserted_ticks[bus] = 0;
    LL_EXTI_GenerateSWI_0_31(BIT(can_int[bus].pin));
    atomic_val_t n = atomic_inc(&st_int_kicks[bus]) + 1;
    if (n <= 5 || (n % 50) == 0) {
        printk("CAN %c: INT stuck asserted, interrupt kicked (%d so far)\n", 'A' + bus, (int)n);
    }
}

int canbus_int_pin(int bus)
{
    return gpio_pin_get_dt(&can_int[bus]);
}

void canbus_get_ctrl_diag(int bus, struct canbus_ctrl_diag *d)
{
    struct can_bus_err_cnt ec = { 0 };
    enum can_state st = CAN_STATE_STOPPED;

    d->state_ok = can_get_state(can_devs[bus], &st, &ec) == 0;
    d->state = (uint8_t)st;
    d->tec = ec.tx_err_cnt;
    d->rec = ec.rx_err_cnt;
    d->restarts = (uint32_t)atomic_get(&st_restarts[bus]);
    d->restart_fail = (uint32_t)atomic_get(&st_restart_fail[bus]);
    d->int_asserted = gpio_pin_get_dt(&can_int[bus]) > 0;
    d->int_kicks = (uint32_t)atomic_get(&st_int_kicks[bus]);
    d->tx_stuck = (uint32_t)atomic_get(&st_tx_stuck[bus]);
}

void canbus_get_stats(struct canbus_stats *out)
{
    for (int b = 0; b < NUM_BUSES; b++) {
        out->rx[b] = atomic_get(&st_rx[b]);
        out->tx_ok[b] = atomic_get(&st_tx_ok[b]);
        out->tx_err[b] = atomic_get(&st_tx_err[b]);
        out->tx_refused[b] = atomic_get(&st_tx_refused[b]);
    }
    out->dup_dropped = atomic_get(&st_dup);
    out->stale_dropped = atomic_get(&st_stale);
    out->rxq_overflow = atomic_get(&st_overflow);
}

const struct device *canbus_dev(int bus)
{
    return can_devs[bus];
}

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

int canbus_init(void)
{
    uint32_t now = k_uptime_get_32();

    for (int bus = 0; bus < NUM_BUSES; bus++) {
        if (!device_is_ready(can_devs[bus]) ||
            !gpio_is_ready_dt(&tp_rx[bus]) || !gpio_is_ready_dt(&tp_tx[bus])) {
            printk("CAN %c: not ready\n", 'A' + bus);
            return -ENODEV;
        }
        gpio_pin_configure_dt(&tp_rx[bus], GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&tp_tx[bus], GPIO_OUTPUT_INACTIVE);

        for (int i = 0; i < ARRAY_SIZE(rx_filters); i++) {
            const struct can_filter flt = {
                .id = rx_filters[i].id,
                .mask = rx_filters[i].mask,
                .flags = 0,
            };
            int id = can_add_rx_filter(can_devs[bus], rx_cb, (void *)(uintptr_t)bus, &flt);
            if (id < 0) {
                printk("CAN %c: filter 0x%03x failed (%d)\n", 'A' + bus, rx_filters[i].id, id);
                return id;
            }
        }

        int err = can_start(can_devs[bus]);
        if (err != 0) {
            printk("CAN %c: start failed (%d)\n", 'A' + bus, err);
            return err;
        }
        atomic_set(&last_rx_ms[bus], (atomic_val_t)now);   /* grace period at boot */
    }

    k_thread_create(&dispatch_thread, dispatch_stack, K_THREAD_STACK_SIZEOF(dispatch_stack),
                    dispatch_fn, NULL, NULL, NULL, PRIO_DISPATCH, 0, K_NO_WAIT);
    k_thread_name_set(&dispatch_thread, "dispatch");
    return 0;
}
