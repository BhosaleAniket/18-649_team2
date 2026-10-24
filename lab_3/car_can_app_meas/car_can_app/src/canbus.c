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
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>

#include <string.h>

#include "canbus.h"
#include "catalog.h"
#include "config.h"
#include "wcet.h"

static const struct device *const can_devs[NUM_BUSES] = {
    DEVICE_DT_GET(DT_NODELABEL(can_a)),
    DEVICE_DT_GET(DT_NODELABEL(can_b)),
};

static const struct gpio_dt_spec tp_rx[NUM_BUSES] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_rx_a), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_rx_b), gpios),
};
static const struct gpio_dt_spec tp_tx[NUM_BUSES] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_tx_a), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(tp_can_tx_b), gpios),
};

/* IDs this zone listens to (everything in the catalog it consumes) */
#if NODE_ID == 2
static const uint16_t rx_ids[] = { ID_CTRL, ID_FAULT, ID_HB_N1, ID_HB_N3, ID_BLINK };
#else
static const uint16_t rx_ids[] = { ID_CTRL, ID_FAULT, ID_HB_N1, ID_HB_N2, ID_BLINK };
#endif

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
    uint32_t t_ms;
    uint8_t bus;
};

K_MSGQ_DEFINE(rx_q, sizeof(struct rx_item), 32, 4);

/* Runs in the MCP2515 interrupt thread (cooperative), not in an ISR. */
static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    uint8_t bus = (uint8_t)(uintptr_t)user_data;
    struct rx_item it = {
        .frame = *frame,
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
        /* MEASUREMENT BUILD: a short pulse instead of a toggle, so every frame gives a
         * rising edge the scope Delay measurement can use (a toggle alternates polarity). */
        gpio_pin_set_dt(&tp_rx[it.bus], 1);     /* CAN_RX_x: dispatch first gets the frame */
        gpio_pin_set_dt(&tp_rx[it.bus], 0);
        WCET_BEGIN(disp);

        const struct can_frame *f = &it.frame;
        if (f->dlc != CAT_DLC || (f->flags & CAN_FRAME_RTR)) {
            continue;                           /* not a catalog frame */
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

        relay_post(f, it.bus);                  /* forwarded on the other bus by RELAY */
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

static int queue_on(int bus, const struct can_frame *f)
{
    gpio_pin_toggle_dt(&tp_tx[bus]);            /* CAN_TX_x: the call that queues the frame */
    WCET_BEGIN(spi);
    /* Wait up to TX_BUFFER_WAIT for one of the 3 MCP2515 TX buffers to free
     * up (one frame on the wire is 270 us), instead of dropping the frame
     * when two or three others are still queued. The completion is still a
     * callback, so this never waits for the frame itself to be sent. */
    int err = can_send(can_devs[bus], f, TX_BUFFER_WAIT, tx_cb, (void *)(uintptr_t)bus);
    WCET_END(W_SPI_TX, spi);
    if (err != 0) {
        atomic_inc(&st_tx_refused[bus]);        /* buffers full (no ACK) or controller stopped */
    }
    return err;
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

void canbus_set_degraded(int bus, bool deg)
{
    bool was = atomic_set(&degraded[bus], deg ? 1 : 0) != 0;

    if (deg && !was) {
        /* Throw away frames stuck in the TX buffers waiting for an ACK,
         * so stale CTRL/HB copies are not sent when the wire comes back.
         * Filters survive stop/start. */
        can_stop(can_devs[bus]);
        can_start(can_devs[bus]);
    }
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

        for (int i = 0; i < ARRAY_SIZE(rx_ids); i++) {
            const struct can_filter flt = {
                .id = rx_ids[i],
                .mask = CAN_STD_ID_MASK,
                .flags = 0,
            };
            int id = can_add_rx_filter(can_devs[bus], rx_cb, (void *)(uintptr_t)bus, &flt);
            if (id < 0) {
                printk("CAN %c: filter 0x%03x failed (%d)\n", 'A' + bus, rx_ids[i], id);
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
