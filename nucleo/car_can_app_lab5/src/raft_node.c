/*
 * raft_node.c - RAFT thread (Lab 5): raft_core + tsync on a Nucleo
 *
 * RAFT (prio PRIO_RAFT): waits on raft_q for at most 1 ms, then
 *   - frames: raft_on_frame(); a REF from the current leader also goes to
 *     tsync_on_ref() with its RX timestamp (taken in the CAN RX callback)
 *   - REF TX-complete stamps: tsync_on_tx_done()
 *   - every 1 ms: inputs from this node's own detection (links from DEG,
 *     peers from heartbeats, cockpit from HB_N1, silent from the test
 *     button), raft_on_tick(), tsync_on_tick() (REF when leader)
 *   - frames to send go to canbus_send_raft()
 * RAFT_LOG (prio PRIO_RAFTLOG): prints applied entries, debug lines and
 *   PPS lines from a queue, so no printk ever runs on the RAFT thread.
 *
 * Outputs: LEADER LED (this node leads), SYNC LED (time reference current:
 * leader, or a REF within SYNC_FRESH_MS), REF_RX test point (toggles on
 * every REF from the leader, direct or relayed).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include "raft_node.h"
#include "raft/route.h"
#include "raft/tsync.h"
#include "raft_config.h"
#include "canbus.h"
#include "catalog.h"
#include "config.h"
#include "state.h"

#define SYNC_FRESH_MS   100
#define OBSERVE_AFTER_MS 300    /* peers' heartbeats need a moment after boot */

static const struct gpio_dt_spec led_leader = GPIO_DT_SPEC_GET(DT_NODELABEL(led_leader), gpios);
static const struct gpio_dt_spec led_sync = GPIO_DT_SPEC_GET(DT_NODELABEL(led_sync), gpios);
static const struct gpio_dt_spec tp_ref_rx = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_ref_rx), gpios);

/* ------------------------------------------------------------------ */
/* State (RAFT thread writes; others read under `lock`)               */
/* ------------------------------------------------------------------ */

static uint8_t raft_log_mem[RAFT_LOG_BYTES(RAFT_CFG_LOG_SIZE)] __aligned(4);
static raft_t raft;
static tsync_t ts;                      /* guarded by lock */
static struct k_spinlock lock;
static raft_net_t pub_net = { 0x07, { 0x03, 0x03, 0x03 } };
static bool pub_failure;
static uint8_t pub_leader, pub_term, pub_role;
static bool pub_is_leader, pub_silent;
static uint16_t pub_last, pub_commit;
static uint64_t last_ref_us;
static uint8_t last_ref_cycle, last_ref_sbus;
static atomic_t st_in, st_out, st_out_drop, st_q_ovf, st_log_drop;

uint64_t raft_node_local_us(void)
{
    return k_cyc_to_us_floor64(k_cycle_get_64());
}

/* ------------------------------------------------------------------ */
/* Queues                                                             */
/* ------------------------------------------------------------------ */

enum { EV_FRAME = 1, EV_TXDONE, EV_TESTFAULT, EV_REPORT };

struct rev {
    uint64_t t;
    uint16_t id;
    uint8_t type;
    uint8_t bus;
    uint8_t arg;
    uint8_t data[8];
};
K_MSGQ_DEFINE(raft_q, sizeof(struct rev), 32, 8);

enum { LOG_APPLY = 1, LOG_TEXT, LOG_PPS };

struct logitem {
    uint8_t type;
    uint8_t term, entry, replayed;
    uint16_t index;
    int32_t a, b;
    uint64_t g, l;
    char text[72];
};
K_MSGQ_DEFINE(log_q, sizeof(struct logitem), 10, 8);

static void post(const struct rev *e)
{
    if (k_msgq_put(&raft_q, e, K_NO_WAIT) != 0) {
        atomic_inc(&st_q_ovf);
    }
}

void raft_node_post_frame(uint8_t bus, const struct can_frame *f, uint64_t rx_us, bool first)
{
    struct rev e = { .t = rx_us, .id = (uint16_t)f->id, .type = EV_FRAME, .bus = bus, .arg = first ? 1 : 0 };
    memcpy(e.data, f->data, 8);
    post(&e);
}

void raft_node_post_ref_txdone(uint8_t bus, uint64_t tx_us)
{
    struct rev e = { .t = tx_us, .type = EV_TXDONE, .bus = bus };
    post(&e);
}

void raft_node_note_test_fault(uint8_t zone)
{
    struct rev e = { .type = EV_TESTFAULT, .arg = zone };
    post(&e);
}

bool raft_node_report_event(uint8_t code)
{
    struct rev e = { .type = EV_REPORT, .arg = RAFT_ENT_MAKE_EVENT(NODE_ID, code) };
    return k_msgq_put(&raft_q, &e, K_NO_WAIT) == 0;
}

static void log_put(const struct logitem *it)
{
    if (k_msgq_put(&log_q, it, K_NO_WAIT) != 0) {
        atomic_inc(&st_log_drop);
    }
}

void raft_node_log_pps(uint64_t global_us, uint64_t local_us);
void raft_node_log_pps(uint64_t global_us, uint64_t local_us)
{
    struct logitem it = { .type = LOG_PPS, .g = global_us, .l = local_us };
    k_spinlock_key_t key = k_spin_lock(&lock);
    tsync_status(&ts, &it.a, &it.b);
    k_spin_unlock(&lock, key);
    log_put(&it);
}

/* ------------------------------------------------------------------ */
/* raft_core callbacks (RAFT thread)                                  */
/* ------------------------------------------------------------------ */

static void on_apply(void *ctx, uint16_t index, uint8_t term, uint8_t entry, bool replayed)
{
    ARG_UNUSED(ctx);
    struct logitem it = { .type = LOG_APPLY, .index = index, .term = term, .entry = entry, .replayed = replayed };
    log_put(&it);
}

static void on_debug(void *ctx, const char *line)
{
    ARG_UNUSED(ctx);
    struct logitem it = { .type = LOG_TEXT };
    strncpy(it.text, line, sizeof(it.text) - 1);
    log_put(&it);
}

/* ------------------------------------------------------------------ */
/* Time and relay (any thread)                                        */
/* ------------------------------------------------------------------ */

uint64_t raft_node_global_us(uint64_t local_us)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint64_t g = tsync_global_us(&ts, local_us);
    k_spin_unlock(&lock, key);
    return g;
}

uint64_t raft_node_global_now(void)
{
    return raft_node_global_us(raft_node_local_us());
}

uint64_t raft_node_local_for_global(uint64_t global_us)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint64_t l = tsync_local_for_global(&ts, global_us);
    k_spin_unlock(&lock, key);
    return l;
}

/* The links this node reports to Raft. DEG trips after 35 ms of silence,
 * which is right for the safety logic, but a bus with one sparse talker
 * (seen on N2's bus B while N3 rebooted: frames every 36-41 ms) toggles
 * DEG every few tens of ms, and every toggle became a LINK DOWN / LINK UP
 * entry pair (idx 56-67 on the bench). For Raft: a link goes down after
 * 100 ms of continuous DEG, and comes back after 500 ms without DEG. */
#define LINK_DOWN_MS 100
#define LINK_UP_MS   500
static uint8_t raft_links = RAFT_BUSBIT(RAFT_BUS_A) | RAFT_BUSBIT(RAFT_BUS_B);
static uint32_t link_since_ms[2];
static bool link_raw_prev[2] = { true, true };

static uint8_t raft_own_links(uint32_t now_ms)
{
    for (int b = 0; b < 2; b++) {
        bool up = !canbus_is_degraded(b == 0 ? BUS_A : BUS_B);
        uint8_t bit = RAFT_BUSBIT(b == 0 ? RAFT_BUS_A : RAFT_BUS_B);
        if (up != link_raw_prev[b]) {
            link_raw_prev[b] = up;
            link_since_ms[b] = now_ms;
        }
        uint32_t held = now_ms - link_since_ms[b];
        if (!up && (raft_links & bit) && held >= LINK_DOWN_MS) {
            raft_links &= (uint8_t)~bit;
        } else if (up && !(raft_links & bit) && held >= LINK_UP_MS) {
            raft_links |= bit;
        }
    }
    return raft_links;
}

static uint8_t own_links(void)
{
    return (uint8_t)((canbus_is_degraded(BUS_A) ? 0 : RAFT_BUSBIT(RAFT_BUS_A)) |
                     (canbus_is_degraded(BUS_B) ? 0 : RAFT_BUSBIT(RAFT_BUS_B)));
}

static bool peer_alive(uint8_t zone, uint32_t now_ms, struct peer_info *p)
{
    state_get_peer(zone, p);
    return p->last_hb_ms != 0 && (int32_t)(now_ms - p->last_hb_ms) < (int32_t)ZONE_TIMEOUT_MS;
}

uint8_t raft_node_relay_bus(uint8_t sender, uint8_t from_bus, bool already_relayed)
{
    uint8_t heard[3];
    uint32_t now_ms = k_uptime_get_32();

    for (uint8_t z = 1; z <= 3; z++) {
        struct peer_info p;
        if (z == NODE_ID) {
            heard[z - 1] = own_links();
        } else {
            heard[z - 1] = peer_alive(z, now_ms, &p) ? (uint8_t)(p.bus_health & 0x03) : 0;
        }
    }
    k_spinlock_key_t key = k_spin_lock(&lock);
    raft_net_t net = pub_net;
    k_spin_unlock(&lock, key);

    uint8_t m = route_relay_mask(&net, heard, NODE_ID, sender, from_bus, already_relayed);
    return m == RAFT_BUSBIT(RAFT_BUS_A) ? RAFT_BUS_A : (m == RAFT_BUSBIT(RAFT_BUS_B) ? RAFT_BUS_B : 0);
}

void raft_node_get_status(struct raft_node_status *s)
{
    uint64_t now = raft_node_local_us();
    k_spinlock_key_t key = k_spin_lock(&lock);
    s->term = pub_term;
    s->leader = pub_leader;
    s->role = pub_role;
    s->is_leader = pub_is_leader;
    s->silent = pub_silent;
    s->last = pub_last;
    s->commit = pub_commit;
    s->net = pub_net;
    s->failure = pub_failure;
    s->synced = tsync_is_synced(&ts);
    tsync_status(&ts, &s->drift_ppb, &s->last_corr_us);
    s->steps = ts.steps;
    s->slews = ts.slews;
    s->no_stamp = ts.no_stamp;
    s->no_match = ts.no_match;
    s->reused = ts.reused;
    s->stale = ts.stale_rejects;
    s->big = ts.big_rejects;
    s->outliers = ts.outliers;
    s->bias_a_us = ts.bias[0];
    s->bias_b_us = ts.bias[1];
    s->last_cycle = last_ref_cycle;
    s->last_sbus = last_ref_sbus;
    s->ref_age_ms = last_ref_us ? (uint32_t)((now - last_ref_us) / 1000u) : UINT32_MAX;
    k_spin_unlock(&lock, key);
    s->ref_fresh = s->is_leader || s->ref_age_ms < SYNC_FRESH_MS;
    s->frames_in = (uint32_t)atomic_get(&st_in);
    s->frames_out = (uint32_t)atomic_get(&st_out);
    s->out_dropped = (uint32_t)atomic_get(&st_out_drop);
}

bool raft_node_failure(void)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    bool f = pub_failure;
    k_spin_unlock(&lock, key);
    return f;
}

/* ------------------------------------------------------------------ */
/* RAFT thread                                                        */
/* ------------------------------------------------------------------ */

static void send_out(raft_out_t *out)
{
    for (int i = 0; i < out->n; i++) {
        if (canbus_send_raft(&out->f[i], false) == 0) {
            atomic_inc(&st_out);
        } else {
            atomic_inc(&st_out_drop);
        }
    }
    out->n = 0;
}

static void publish(void)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    pub_net = raft_get_network(&raft);
    pub_failure = raft_failure_mode(&raft);
    pub_leader = raft_leader(&raft);
    pub_term = raft_term(&raft);
    pub_role = raft_role(&raft);
    pub_is_leader = raft_is_leader(&raft);
    pub_silent = raft.silent;
    pub_last = raft_last_index(&raft);
    pub_commit = raft_commit_index(&raft);
    k_spin_unlock(&lock, key);
}

static uint8_t prev_leader;

static void check_leader_change(void)
{
    uint8_t l = raft_leader(&raft);
    if (l != prev_leader) {
        if (prev_leader != 0) {
            k_spinlock_key_t key = k_spin_lock(&lock);
            tsync_on_leader_change(&ts);
            k_spin_unlock(&lock, key);
        }
        prev_leader = l;
    }
}

static void handle(const struct rev *e, raft_out_t *out)
{
    switch (e->type) {
    case EV_FRAME: {
        bool first = e->arg != 0;
        uint8_t sender = raft_id_sender(e->id);
        if (first) {
            atomic_inc(&st_in);
            raft_on_frame(&raft, e->bus, e->id, e->data, e->t, out);
            check_leader_change();
        }
        /* Every copy of a REF from the leader goes to tsync (it pairs the
         * copy on the stamped bus); REF_RX toggles once per REF */
        if (raft_id_base(e->id) == RAFT_ID_REF && raft_leader(&raft) == sender && !raft_is_leader(&raft)) {
            k_spinlock_key_t key = k_spin_lock(&lock);
            tsync_on_ref(&ts, e->bus, e->data, (e->data[0] & RAFT_FLAG_RELAYED) != 0, e->t);
            if (first) {
                last_ref_us = e->t;
                last_ref_cycle = e->data[2];
                last_ref_sbus = e->data[3] & 0x03;
            }
            k_spin_unlock(&lock, key);
            if (first) {
                gpio_pin_toggle_dt(&tp_ref_rx);
            }
        }
        break;
    }
    case EV_TXDONE: {
        k_spinlock_key_t key = k_spin_lock(&lock);
        tsync_on_tx_done(&ts, e->bus, e->t);
        k_spin_unlock(&lock, key);
        break;
    }
    case EV_TESTFAULT:
        raft_note_test_fault(&raft, e->arg);
        break;
    case EV_REPORT:
        raft_report(&raft, e->arg);
        break;
    default:
        break;
    }
}

static void tick(uint64_t now, raft_out_t *out)
{
    uint32_t now_ms = k_uptime_get_32();

    raft_set_own_links(&raft, raft_own_links(now_ms));
    raft_set_silent(&raft, (state_reasons() & ERR_TEST) != 0);
    if (now_ms > OBSERVE_AFTER_MS) {
        uint8_t alive = 0;
        struct peer_info cockpit = { 0 };
        for (uint8_t z = 1; z <= 3; z++) {
            struct peer_info p;
            if (z != NODE_ID && peer_alive(z, now_ms, &p)) {
                alive |= RAFT_NODEBIT(z);
                if (z == ZONE_N1) {
                    cockpit = p;
                }
            }
        }
        raft_observe_peers(&raft, alive);
        /* COCKPIT follows HB_N1: present and reporting normal */
        raft_observe_cockpit(&raft, (alive & RAFT_NODEBIT(ZONE_N1)) && cockpit.zone_state == 1);
    }

    raft_on_tick(&raft, now, out);
    check_leader_change();
    send_out(out);

    raft_frame_t ref;
    k_spinlock_key_t key = k_spin_lock(&lock);
    bool send_ref = tsync_on_tick(&ts, raft_is_leader(&raft), raft_term(&raft), now, &ref);
    k_spin_unlock(&lock, key);
    if (send_ref) {
        if (canbus_send_raft(&ref, true) == 0) {
            atomic_inc(&st_out);
        } else {
            atomic_inc(&st_out_drop);
        }
    }

    publish();
    struct raft_node_status s;
    raft_node_get_status(&s);
    gpio_pin_set_dt(&led_leader, s.is_leader);
    gpio_pin_set_dt(&led_sync, s.synced && s.ref_fresh && !s.silent);
}

K_THREAD_STACK_DEFINE(raft_stack, STACK_RAFT);
static struct k_thread raft_thread;

static void raft_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    raft_out_t out = { .n = 0 };
    uint64_t next_tick = raft_node_local_us() + 1000;

    while (1) {
        uint64_t now = raft_node_local_us();
        k_timeout_t to = now >= next_tick ? K_NO_WAIT : K_USEC((uint32_t)(next_tick - now));
        struct rev e;

        if (k_msgq_get(&raft_q, &e, to) == 0) {
            handle(&e, &out);
            send_out(&out);
        }
        now = raft_node_local_us();
        if (now >= next_tick) {
            next_tick += 1000;
            if (next_tick <= now) {
                next_tick = now + 1000;
            }
            tick(now, &out);
        }
    }
}

/* ------------------------------------------------------------------ */
/* RAFT_LOG thread (printk only here)                                 */
/* ------------------------------------------------------------------ */

K_THREAD_STACK_DEFINE(raftlog_stack, STACK_MEDIUM);
static struct k_thread raftlog_thread;

static void raftlog_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    struct logitem it;
    char s[48];

    while (1) {
        k_msgq_get(&log_q, &it, K_FOREVER);
        switch (it.type) {
        case LOG_APPLY:
            raft_entry_str(it.entry, s, sizeof(s));
            printk("RAFT idx %u term %u %s%s\n", it.index, it.term, s, it.replayed ? " (replayed)" : "");
            break;
        case LOG_TEXT:
            printk("%s\n", it.text);
            break;
        case LOG_PPS:
            printk("PPS global=%llu.%06llu s local=%llu.%06llu s drift=%d ppb corr=%d us\n",
                   it.g / 1000000u, it.g % 1000000u, it.l / 1000000u, it.l % 1000000u, it.a, it.b);
            break;
        default:
            break;
        }
    }
}

int raft_node_init(void)
{
    const struct gpio_dt_spec *pins[] = { &led_leader, &led_sync, &tp_ref_rx };
    for (int i = 0; i < ARRAY_SIZE(pins); i++) {
        if (!gpio_is_ready_dt(pins[i])) {
            return -ENODEV;
        }
        gpio_pin_configure_dt(pins[i], GPIO_OUTPUT_INACTIVE);
    }

    raft_config_t cfg = {
        .n_nodes = RAFT_CFG_N_NODES,
        .self_id = NODE_ID,
        .default_leader = RAFT_CFG_DEFAULT_LEADER,
        .heartbeat_ms = RAFT_CFG_HEARTBEAT_MS,
        .append_period_ms = RAFT_CFG_APPEND_PERIOD_MS,
        .boot_listen_ms = RAFT_CFG_BOOT_LISTEN_MS,
        .missed_heartbeats = RAFT_CFG_MISSED_HEARTBEATS,
        .election_jitter_ms = RAFT_CFG_ELECTION_JITTER_MS,
        .catchup_pace_ms = RAFT_CFG_CATCHUP_PACE_MS,
        .report_retry_ms = RAFT_CFG_REPORT_RETRY_MS,
        .election_seed = RAFT_CFG_ELECTION_SEED,
        .log_size = RAFT_CFG_LOG_SIZE,
        .debug = RAFT_CFG_DEBUG,
    };
    uint64_t now = raft_node_local_us();
    raft_init(&raft, &cfg, raft_log_mem, now, on_apply, on_debug, NULL);
    tsync_init(&ts, NODE_ID, RAFT_CFG_RX_LATENCY_US, RAFT_CFG_RELAY_DELAY_US);
    tsync_set_period_ms(&ts, RAFT_CFG_HEARTBEAT_MS);
    publish();

    printk("raft: catalog v%s, node %d, default leader %s, log %u entries, seed %u, %s\n",
           RAFT_CFG_CATALOG_VERSION, NODE_ID, RAFT_CFG_DEFAULT_LEADER_NAME, RAFT_CFG_LOG_SIZE,
           (unsigned)RAFT_CFG_ELECTION_SEED, RAFT_CFG_DEBUG ? "debug on" : "debug off");

    k_thread_create(&raftlog_thread, raftlog_stack, K_THREAD_STACK_SIZEOF(raftlog_stack),
                    raftlog_fn, NULL, NULL, NULL, PRIO_RAFTLOG, 0, K_NO_WAIT);
    k_thread_name_set(&raftlog_thread, "raft_log");
    k_thread_create(&raft_thread, raft_stack, K_THREAD_STACK_SIZEOF(raft_stack),
                    raft_fn, NULL, NULL, NULL, PRIO_RAFT, 0, K_NO_WAIT);
    k_thread_name_set(&raft_thread, "raft");
    return 0;
}
