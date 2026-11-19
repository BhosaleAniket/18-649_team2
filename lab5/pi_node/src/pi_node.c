/*
 * pi_node.c - the Pi's C runtime (catalog v0.7, "On the Pi").
 *
 * Owns canA and canB. Threads (SCHED_FIFO when allowed):
 *   rx[2]   one per bus: kernel receive timestamp, TX-confirm of our own REF, dedup
 *           (Raft and Lab 3 rules), per-bus / per-zone detection, relay, hand-off
 *   tick    every 1 ms: Raft core + time sync, REF, and every 20 ms the monitor
 *           (DEG, zones, HB_N1, LINK/NODE/COCKPIT/test-button reports)
 *   log     low priority: prints messages queued by the others (never print in RT paths)
 *
 * cockpit.py talks to it only through pi_node.h (ctypes). Lab 3 frames from the other zones
 * come back through pi_node_poll_app(); CTRL and BLINK go out through pi_node_send_app().
 *
 * Lock order: core_mx before st_mx. tx_mx is a leaf.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <net/if.h>
#include <pthread.h>
#include <sched.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "pi_node.h"
#include "catalog.h"
#include "route.h"
#include "tsync.h"
#include "config.h"
#include "gpio.h"

#define NBUS               2
#define DEG_TIMEOUT_US     35000u     /* Lab 3: no frame on a bus for 35 ms -> degraded */
#define ZONE_TIMEOUT_US    66000u     /* Lab 3: no heartbeat on either bus for 66 ms -> zone lost */
#define DEDUP_US           150000u
#define STALE_US           250000u
#define MON_PERIOD_US      20000u     /* HB_N1 and the monitor */
#define OBSERVE_AFTER_US   300000u    /* as on the Nucleos: peers' heartbeats need a moment after boot */

static const char *const IFACE[NBUS] = { "canA", "canB" };

/* ------------------------------------------------------------------------------------------
 * Small thread-safe queue
 * ------------------------------------------------------------------------------------------ */
typedef struct {
    pthread_mutex_t mx;
    pthread_cond_t cv;
    uint8_t *buf;
    size_t item, cap, head, n;
    uint32_t dropped;
} ring_t;

static volatile int g_running;

static int ring_init(ring_t *r, size_t item, size_t cap)
{
    memset(r, 0, sizeof *r);
    r->buf = calloc(cap, item);
    if (!r->buf) return -1;
    r->item = item;
    r->cap = cap;
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&r->cv, &ca);
    pthread_condattr_destroy(&ca);
    pthread_mutex_init(&r->mx, NULL);
    return 0;
}

static void ring_free(ring_t *r)
{
    free(r->buf);
    r->buf = NULL;
}

static void ring_push(ring_t *r, const void *it)        /* full: drop the oldest */
{
    pthread_mutex_lock(&r->mx);
    if (r->n == r->cap) {
        r->head = (r->head + 1) % r->cap;
        r->n--;
        r->dropped++;
    }
    memcpy(r->buf + ((r->head + r->n) % r->cap) * r->item, it, r->item);
    r->n++;
    pthread_cond_signal(&r->cv);
    pthread_mutex_unlock(&r->mx);
}

/* 0 = got one, 1 = timeout. timeout_ms < 0 waits until pi_node_stop(). */
static int ring_pop(ring_t *r, void *out, int timeout_ms)
{
    pthread_mutex_lock(&r->mx);
    if (r->n == 0 && timeout_ms != 0) {
        struct timespec dl;
        clock_gettime(CLOCK_MONOTONIC, &dl);
        long ms = timeout_ms < 0 ? 200 : timeout_ms;
        dl.tv_sec += ms / 1000;
        dl.tv_nsec += (ms % 1000) * 1000000L;
        if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
        while (r->n == 0) {
            int rc = pthread_cond_timedwait(&r->cv, &r->mx, &dl);
            if (rc == ETIMEDOUT) {
                if (timeout_ms >= 0 || !g_running) break;
                dl.tv_nsec += 200000000L;
                if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
            }
        }
    }
    if (r->n == 0) {
        pthread_mutex_unlock(&r->mx);
        return 1;
    }
    memcpy(out, r->buf + r->head * r->item, r->item);
    r->head = (r->head + 1) % r->cap;
    r->n--;
    pthread_mutex_unlock(&r->mx);
    return 0;
}

/* ------------------------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------------------------ */
enum { EV_FRAME = 1, EV_TESTFAULT };
typedef struct { uint16_t id; uint8_t d[8]; uint8_t type, bus, first, relayed; uint64_t t; } rx_item_t;
typedef struct { uint16_t id; uint8_t d[8]; uint8_t bus; } app_item_t;
typedef struct { uint16_t index; uint8_t term, entry, replayed; } log_item_t;
typedef struct { char s[200]; } msg_item_t;

static struct {
    raft_config_t rc;
    pi_extra_t px;
    uint8_t self;
    int sock[NBUS];
    pthread_t th_rx[NBUS], th_tick, th_log, th_pps;
    bool pps_pin_ok;
    int n_threads_started;

    pthread_mutex_t core_mx;            /* raft + ts */
    raft_t *raft;
    uint8_t *logmem;                    /* RAFT_LOG_BYTES(log_size), handed to raft_init() */
    tsync_t *ts;
    uint64_t boot_us;

    pthread_mutex_t st_mx;              /* everything below, up to tx_mx */
    uint64_t last_rx[NBUS];
    bool degraded[NBUS];
    uint8_t own_links;                  /* bit (bus-1) = not degraded */
    uint64_t hb_time[3];
    uint8_t hb_bits[3];                 /* byte 2 of node n's latest heartbeat */
    uint8_t zones_alive;                /* bit (n-1) */
    raft_net_t net_cache;               /* committed graph, refreshed every tick */
    bool cockpit_ok;
    uint8_t err_bits;
    struct { uint16_t id; uint8_t src, seq, used; uint64_t t; } seen[32];
    struct { uint8_t key[10]; uint64_t t; } rdup[64];
    unsigned rdup_head;
    uint32_t dups, stale;

    pthread_mutex_t tx_mx;
    uint8_t hb_seq;
    uint8_t seq_raft[8];                /* v0.8: the CAN layer fills byte 0's seq, one counter per Raft base */
    uint32_t refused[NBUS], relayed_n;

    ring_t rxq, appq, logq, msgq;
} g;

/* Age of an event at time `then`, seen from `now`. Frames are stamped by the kernel in the RX
 * threads, so an event can be a few us LATER than a `now` taken just before: that is age 0,
 * never an unsigned wrap to ~2^64 (which made a live bus or zone look dead). */
static inline uint64_t age_us(uint64_t now, uint64_t then)
{
    return now > then ? now - then : 0;
}

static uint64_t now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}

/* ------------------------------------------------------------------------------------------
 * Logging (queued; printed by the log thread)
 * ------------------------------------------------------------------------------------------ */
static void node_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void node_log(const char *fmt, ...)
{
    msg_item_t m;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m.s, sizeof m.s, fmt, ap);
    va_end(ap);
    if (g.msgq.buf) ring_push(&g.msgq, &m);
    else fprintf(stderr, "%s\n", m.s);
}

static void *log_main(void *arg)
{
    (void)arg;
    msg_item_t m;
    while (g_running || g.msgq.n) {
        if (ring_pop(&g.msgq, &m, 200) == 0) {
            fputs(m.s, stdout);
            fputc('\n', stdout);
            fflush(stdout);
        }
    }
    return NULL;
}


/* ------------------------------------------------------------------------------------------
 * Transmit
 * ------------------------------------------------------------------------------------------ */
static void send_frame(uint8_t mask, uint16_t id, const uint8_t d[8])
{
    struct can_frame f;
    memset(&f, 0, sizeof f);
    f.can_id = id;
    f.can_dlc = 8;
    memcpy(f.data, d, 8);
    pthread_mutex_lock(&g.tx_mx);
    for (int b = 0; b < NBUS; b++) {
        if (!(mask & (1u << b))) continue;
        if (send(g.sock[b], &f, sizeof f, MSG_DONTWAIT) != (ssize_t)sizeof f)
            g.refused[b]++;               /* TX queue full: nobody ACKing on that bus */
    }
    pthread_mutex_unlock(&g.tx_mx);
}

static uint8_t own_links(void)
{
    return __atomic_load_n(&g.own_links, __ATOMIC_RELAXED);
}

/* A Raft frame from the core or tsync: fill the seq nibble (one counter per base, bumped once
 * for both copies, as canbus_send_raft() on the Nucleos) and send on our own up links only. */
static void send_raft(const raft_frame_t *rf)
{
    uint16_t base = raft_id_base(rf->id);
    int k = (base == RAFT_ID_REF) ? 0 : (int)((base >> 4) & 0x07);
    uint8_t d[8];
    memcpy(d, rf->data, 8);
    pthread_mutex_lock(&g.tx_mx);
    uint8_t seq = g.seq_raft[k]++;
    pthread_mutex_unlock(&g.tx_mx);
    d[0] = (uint8_t)((d[0] & 0xF0) | (seq & 0x0F));
    uint8_t m = rf->bus_mask & own_links();
    if (m) send_frame(m, rf->id, d);
}

static void send_out(raft_out_t *out)
{
    for (uint8_t i = 0; i < out->n; i++) send_raft(&out->f[i]);
    out->n = 0;
}

/* TX-queue flush (as Lab 3) and interface watchdog. `ip link set up` can time out on the
 * MCP2515 (seen on the bench: "RTNETLINK answers: Connection timed out"), which used to leave
 * the interface down for good. Now: one flush per bus at a time, `up` is retried for 2 s, and
 * monitor() brings any interface it finds down back up. */
static int flushing[NBUS];
enum { FLUSH = 0, BRING_UP = 1 };

static bool iface_is_up(int b)
{
    char path[64];
    unsigned flags = 0;
    snprintf(path, sizeof path, "/sys/class/net/%s/flags", IFACE[b]);
    FILE *f = fopen(path, "r");
    if (!f) return false;
    if (fscanf(f, "%x", &flags) != 1) flags = 0;
    fclose(f);
    return (flags & 0x1) != 0;                   /* IFF_UP */
}

static void *flush_main(void *arg)
{
    int b = (int)(intptr_t)arg & 0xF;
    int mode = (int)(intptr_t)arg >> 4;
    if (__atomic_exchange_n(&flushing[b], 1, __ATOMIC_ACQ_REL)) return NULL;   /* one at a time */
    char cmd[200];
    snprintf(cmd, sizeof cmd,
             "%s%s%s for i in 1 2 3 4 5 6 7 8 9 10; do ip link set %s up 2>/dev/null && exit 0; sleep 0.2; done; exit 1",
             mode == FLUSH ? "ip link set " : "", mode == FLUSH ? IFACE[b] : "",
             mode == FLUSH ? " down;" : "", IFACE[b]);
    if (system(cmd) != 0 || !iface_is_up(b))
        node_log("[can] %s: could not bring the interface up (run as root?)", IFACE[b]);
    else if (mode == BRING_UP)
        node_log("[can] %s was down: brought back up", IFACE[b]);
    __atomic_store_n(&flushing[b], 0, __ATOMIC_RELEASE);
    return NULL;
}

static void spawn_flush(int b, int mode)
{
    pthread_t th;
    if (pthread_create(&th, NULL, flush_main, (void *)(intptr_t)(b | (mode << 4))) == 0) pthread_detach(th);
}

/* ------------------------------------------------------------------------------------------
 * Dedup (st_mx held)
 * ------------------------------------------------------------------------------------------ */
enum { ACCEPT = 0, DUP = 1, STALE = 2 };

static int lab3_check(uint16_t id, uint8_t src, uint8_t seq, uint64_t t)
{
    int free_i = -1, old_i = 0;
    for (int i = 0; i < 32; i++) {
        if (!g.seen[i].used) { if (free_i < 0) free_i = i; continue; }
        if (g.seen[i].t < g.seen[old_i].t) old_i = i;
        if (g.seen[i].id != id || g.seen[i].src != src) continue;
        uint8_t ahead = (uint8_t)(seq - g.seen[i].seq) & 0x0F;
        uint64_t age = age_us(t, g.seen[i].t);
        if (ahead == 0 && age < DEDUP_US) return DUP;
        if (ahead >= 9 && age < STALE_US) return STALE;
        g.seen[i].seq = seq;
        g.seen[i].t = t;
        return ACCEPT;
    }
    int i = free_i >= 0 ? free_i : old_i;
    g.seen[i].id = id; g.seen[i].src = src; g.seen[i].seq = seq; g.seen[i].t = t; g.seen[i].used = 1;
    return ACCEPT;
}

/* v0.7: key = (ID, byte 0 low nibble, bytes 1-7) within 150 ms, flags masked */
static bool raft_dup(uint16_t id, const uint8_t d[8], uint64_t t)
{
    uint8_t key[10] = { (uint8_t)(id & 0xFF), (uint8_t)(id >> 8), (uint8_t)(d[0] & 0x0F),
                        d[1], d[2], d[3], d[4], d[5], d[6], d[7] };
    for (int i = 0; i < 64; i++)
        if (g.rdup[i].t && age_us(t, g.rdup[i].t) < DEDUP_US && !memcmp(g.rdup[i].key, key, 10))
            return true;
    memcpy(g.rdup[g.rdup_head].key, key, 10);
    g.rdup[g.rdup_head].t = t;
    g.rdup_head = (g.rdup_head + 1) % 64;
    return false;
}

/* ------------------------------------------------------------------------------------------
 * Receive
 * ------------------------------------------------------------------------------------------ */
static void heard_snapshot(uint8_t heard[3], uint64_t t)   /* st_mx held */
{
    for (int n = 0; n < 3; n++)
        heard[n] = (g.hb_time[n] && age_us(t, g.hb_time[n]) < ZONE_TIMEOUT_US) ? g.hb_bits[n] : 0;
    heard[g.self - 1] = g.own_links;                    /* our own bus-health bits */
}

static void handle_tx_confirm(int b, const struct can_frame *f, uint64_t t)
{
    if ((f->can_id & CAN_SFF_MASK) != (canid_t)(RAFT_ID_REF + g.self)) return;
    pthread_mutex_lock(&g.core_mx);
    tsync_on_tx_done(g.ts, (uint8_t)(b + 1), t - (uint64_t)(int64_t)g.px.ref_stamp_adjust_us);
    pthread_mutex_unlock(&g.core_mx);
}

static void handle_rx(int b, const struct can_frame *f, uint64_t t)
{
    uint16_t id = f->can_id & CAN_SFF_MASK;
    const uint8_t *d = f->data;
    uint8_t sender = cat_source(id, d);
    if (sender < 1 || sender > 3 || sender == g.self) return;   /* unknown, or our own relayed back */

    bool raft = cat_is_raft(id);
    bool is_ref = raft && raft_id_base(id) == RAFT_ID_REF;
    bool relayed = raft && (d[0] & RAFT_FLAG_RELAYED);
    uint8_t relay_mask = 0;
    bool dup, test_fault = false;

    pthread_mutex_lock(&g.st_mx);
    g.last_rx[b] = t;
    if (raft) {
        dup = raft_dup(id, d, t);
    } else {
        int v = lab3_check(id, sender, d[0] & 0x0F, t);
        if (v == DUP) g.dups++;
        if (v == STALE) g.stale++;
        dup = v != ACCEPT;
    }
    if (!dup) {
        if (id == ID_HB_N1 || id == ID_HB_N2 || id == ID_HB_N3) {
            g.hb_time[sender - 1] = t;
            g.hb_bits[sender - 1] = d[2];
        } else if (id == ID_FAULT && d[2] == FAULT_TEST_BUTTON && d[1] >= 1 && d[1] <= 3) {
            test_fault = true;                        /* the core logs it as APP_EVENT once */
        }
        uint8_t heard[3];
        heard_snapshot(heard, t);
        relay_mask = route_relay_mask(&g.net_cache, heard, g.self, sender, (uint8_t)(b + 1), relayed);
    }
    pthread_mutex_unlock(&g.st_mx);

    /* REF: tsync pairs on the stamp bus, so it gets every copy (both buses); the core only the
     * first. (The Nucleos' canbus.c drops the second copy before tsync sees it.) */
    if (is_ref) {
        rx_item_t it = { id, {0}, EV_FRAME, (uint8_t)(b + 1), (uint8_t)!dup, relayed, t };
        memcpy(it.d, d, 8);
        ring_push(&g.rxq, &it);
    }
    if (dup) return;
    if (test_fault) {
        rx_item_t it = { id, {0}, EV_TESTFAULT, (uint8_t)(b + 1), d[1], 0, t };
        ring_push(&g.rxq, &it);
    }
    if (raft && !is_ref) {
        rx_item_t it = { id, {0}, EV_FRAME, (uint8_t)(b + 1), 1, relayed, t };
        memcpy(it.d, d, 8);
        ring_push(&g.rxq, &it);
    } else if (!raft) {
        app_item_t a = { id, {0}, (uint8_t)(b + 1) };
        memcpy(a.d, d, 8);
        ring_push(&g.appq, &a);
    }
    if (relay_mask) {
        uint8_t c[8];
        memcpy(c, d, 8);
        if (raft) c[0] |= RAFT_FLAG_RELAYED;              /* Lab 3 frames stay bit for bit (CRC) */
        send_frame(relay_mask, id, c);
        __atomic_add_fetch(&g.relayed_n, 1, __ATOMIC_RELAXED);
    }
}

static void *rx_main(void *arg)
{
    int b = (int)(intptr_t)arg;
    struct can_frame f;
    char ctrl[CMSG_SPACE(sizeof(struct timespec)) + 64];
    while (g_running) {
        struct iovec iov = { &f, sizeof f };
        struct msghdr m;
        memset(&m, 0, sizeof m);
        m.msg_iov = &iov;
        m.msg_iovlen = 1;
        m.msg_control = ctrl;
        m.msg_controllen = sizeof ctrl;
        ssize_t n = recvmsg(g.sock[b], &m, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            usleep(10000);                           /* interface being flushed / down */
            continue;
        }
        if (n < (ssize_t)sizeof f) continue;

        /* kernel receive stamp (CLOCK_REALTIME) -> our monotonic microseconds */
        struct timespec rt, mono, ks;
        bool have_ks = false;
        for (struct cmsghdr *c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c))
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
                memcpy(&ks, CMSG_DATA(c), sizeof ks);
                have_ks = true;
            }
        clock_gettime(CLOCK_REALTIME, &rt);
        clock_gettime(CLOCK_MONOTONIC, &mono);
        uint64_t t;
        if (have_ks) {
            int64_t off_ns = ((int64_t)rt.tv_sec - mono.tv_sec) * 1000000000LL + (rt.tv_nsec - mono.tv_nsec);
            int64_t k_ns = (int64_t)ks.tv_sec * 1000000000LL + ks.tv_nsec;
            t = (uint64_t)((k_ns - off_ns) / 1000);
        } else {
            t = (uint64_t)mono.tv_sec * 1000000u + (uint64_t)mono.tv_nsec / 1000u;
        }

        if (f.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) continue;
        if (f.can_dlc != 8) continue;
        if (m.msg_flags & MSG_CONFIRM) {             /* our own frame, back from the controller */
            handle_tx_confirm(b, &f, t);
            continue;
        }
        handle_rx(b, &f, t);
    }
    return NULL;
}

/* ------------------------------------------------------------------------------------------
 * Raft apply callback (called by the core, core_mx held by the tick thread)
 * ------------------------------------------------------------------------------------------ */
static void on_apply(void *ctx, uint16_t index, uint8_t term, uint8_t entry, bool replayed)
{
    (void)ctx;
    log_item_t li = { index, term, entry, (uint8_t)replayed };
    ring_push(&g.logq, &li);
    if (g.rc.debug) {
        char s[64];
        raft_entry_str(entry, s, sizeof s);
        node_log("RAFT idx %u term %u %s%s", index, term, s, replayed ? " (replayed)" : "");
    }
}

static void on_debug(void *ctx, const char *line)
{
    (void)ctx;
    node_log("%s", line);
}

/* ------------------------------------------------------------------------------------------
 * Tick thread
 * ------------------------------------------------------------------------------------------ */
static uint8_t ts_prev_leader;                     /* core_mx held */

static void check_leader_change(void)              /* core_mx held; as raft_node.c on the Nucleos */
{
    uint8_t l = raft_leader(g.raft);
    if (l != ts_prev_leader) {
        if (ts_prev_leader != 0) tsync_on_leader_change(g.ts);
        ts_prev_leader = l;
    }
}

/* Every 20 ms: DEG + zones (Lab 3), HB_N1, and this node's observations for the core. */
static void monitor(uint64_t t)
{
    uint8_t links, zones, err, hb_seq;
    bool ok, flush[NBUS] = { false, false };

    pthread_mutex_lock(&g.st_mx);
    for (int b = 0; b < NBUS; b++) {
        bool deg = age_us(t, g.last_rx[b]) >= DEG_TIMEOUT_US;
        if (deg != g.degraded[b]) {
            g.degraded[b] = deg;
            flush[b] = deg;
            node_log("[deg] %s %s", IFACE[b], deg ? "DEGRADED" : "ok");
        }
    }
    links = (uint8_t)((g.degraded[0] ? 0 : 1) | (g.degraded[1] ? 0 : 2));
    __atomic_store_n(&g.own_links, links, __ATOMIC_RELAXED);
    zones = 0;
    for (int n = 0; n < 3; n++) {
        if (n == g.self - 1) continue;
        bool alive = g.hb_time[n] && age_us(t, g.hb_time[n]) < ZONE_TIMEOUT_US;
        bool was = g.zones_alive & (1u << n);
        if (alive != was) node_log("[zone] %s %s", raft_node_name((uint8_t)(n + 1)), alive ? "heard" : "LOST");
        if (alive) zones |= (uint8_t)(1u << n);
    }
    g.zones_alive = zones;
    ok = g.cockpit_ok;
    err = g.err_bits;
    pthread_mutex_unlock(&g.st_mx);

    static unsigned watchdog_ticks;
    bool check_up = (++watchdog_ticks % 50u) == 0;   /* once a second */
    for (int b = 0; b < NBUS; b++) {
        if (flush[b])
            spawn_flush(b, FLUSH);
        else if (check_up && !__atomic_load_n(&flushing[b], __ATOMIC_ACQUIRE) && !iface_is_up(b))
            spawn_flush(b, BRING_UP);
    }

    /* HB_N1 (Lab 3 layout), both buses, none while our own test-button fault is active */
    if (!(err & ERR_TEST)) {
        uint8_t d[8] = { 0 };
        pthread_mutex_lock(&g.tx_mx);
        hb_seq = g.hb_seq = (uint8_t)((g.hb_seq + 1) & 0x0F);
        pthread_mutex_unlock(&g.tx_mx);
        d[0] = hb_seq;
        d[1] = (ok && err == 0) ? 1 : 0;
        d[2] = links;
        d[3] = err;
        uint8_t heard_bits = (uint8_t)(1u << g.self);  /* bit n = zone n heard */
        for (int n = 1; n <= 3; n++)
            if (zones & (1u << (n - 1))) heard_bits |= (uint8_t)(1u << n);
        d[4] = heard_bits;
        send_frame(0x3, ID_HB_N1, d);
    }

    /* The core writes LINK (own links), NODE, COCKPIT and test-button entries itself */
    pthread_mutex_lock(&g.core_mx);
    raft_set_own_links(g.raft, links);
    raft_set_silent(g.raft, (err & ERR_TEST) != 0);
    if (age_us(t, g.boot_us) > OBSERVE_AFTER_US) {
        raft_observe_peers(g.raft, zones);
        raft_observe_cockpit(g.raft, ok && !(err & ERR_TEST));   /* the Pi is the cockpit */
    }
    pthread_mutex_unlock(&g.core_mx);
}

static void *tick_main(void *arg)
{
    (void)arg;
    uint64_t next = now_us(), next_mon = next;
    uint8_t prev_leader = 0, prev_term = 0;
    raft_out_t out = { .n = 0 };
    struct timespec ts;
    while (g_running) {
        next += 1000;
        ts.tv_sec = (time_t)(next / 1000000u);
        ts.tv_nsec = (long)(next % 1000000u) * 1000L;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        uint64_t t = now_us();
        if (age_us(t, next) > 20000) next = t;           /* stalled: resync, never burst */

        rx_item_t it;
        while (ring_pop(&g.rxq, &it, 0) == 0) {
            pthread_mutex_lock(&g.core_mx);
            if (it.type == EV_TESTFAULT) {
                raft_note_test_fault(g.raft, it.first);  /* `first` carries the failing zone */
            } else {
                uint8_t sender = raft_id_sender(it.id);
                if (it.first) {
                    raft_on_frame(g.raft, it.bus, it.id, it.d, it.t, &out);
                    check_leader_change();
                }
                /* REF from the current leader (every copy, both buses) -> time sync */
                if (raft_id_base(it.id) == RAFT_ID_REF && raft_leader(g.raft) == sender &&
                    !raft_is_leader(g.raft)) {
                    /* The Pi's two MCP2515s report a frame's arrival with different delays;
                     * pi.rx_skew_b_us lines bus B up with bus A (the stamp bus alternates). */
                    uint64_t rx = it.t;
                    if (it.bus == 2) rx += (uint64_t)(int64_t)g.px.rx_skew_b_us;
                    uint32_t before = g.ts->steps + g.ts->slews;
                    tsync_on_ref(g.ts, it.bus, it.d, it.relayed, rx);
                    if (g.px.tsync_trace && g.ts->steps + g.ts->slews != before)
                        node_log("TSYNC rx bus %c stamp bus %c corr %d us drift %d ppb%s",
                                 it.bus == 1 ? 'A' : 'B', (it.d[3] & 3) == 1 ? 'A' : 'B',
                                 g.ts->last_corr, g.ts->ppb, it.relayed ? " relayed" : "");
                }
            }
            pthread_mutex_unlock(&g.core_mx);
            send_out(&out);
        }

        if (t >= next_mon) {
            next_mon += MON_PERIOD_US;
            if (age_us(t, next_mon) > MON_PERIOD_US) next_mon = t + MON_PERIOD_US;
            monitor(t);
        }

        raft_frame_t ref;
        bool send_ref;
        uint8_t leader, term;
        raft_net_t net;
        pthread_mutex_lock(&g.core_mx);
        raft_on_tick(g.raft, t, &out);
        check_leader_change();
        term = raft_term(g.raft);
        send_ref = tsync_on_tick(g.ts, raft_is_leader(g.raft), term, t, &ref);
        leader = raft_leader(g.raft);
        net = raft_get_network(g.raft);
        pthread_mutex_unlock(&g.core_mx);

        send_out(&out);
        if (send_ref) send_raft(&ref);

        if (leader != prev_leader || term != prev_term) {
            node_log("[raft] term %u leader %s%s", term, leader ? raft_node_name(leader) : "none",
                     leader == g.self ? " (this node)" : "");
            prev_leader = leader;
            prev_term = term;
        }
        pthread_mutex_lock(&g.st_mx);
        g.net_cache = net;
        pthread_mutex_unlock(&g.st_mx);
    }
    return NULL;
}

/* ------------------------------------------------------------------------------------------
 * PPS (Lab 5 Part 6.4): a rising edge on pi.pps_gpio at every whole second of global time
 * ------------------------------------------------------------------------------------------ */
#define PPS_WAKE_EARLY_US  300u       /* sleep until this long before the edge, then spin */
#define PPS_HIGH_US        100000u    /* pulse width */
#define PPS_LATE_LIMIT_US  1000u      /* an edge we woke up for more than this late is skipped */

uint64_t pi_node_global_us(void)               /* global_gettime() */
{
    if (!g_running) return 0;
    pthread_mutex_lock(&g.core_mx);
    uint64_t gl = tsync_global_us(g.ts, now_us());
    pthread_mutex_unlock(&g.core_mx);
    return gl;
}

static void sleep_until_us(uint64_t t)
{
    struct timespec ts = { (time_t)(t / 1000000u), (long)(t % 1000000u) * 1000L };
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* The edge time is re-read after every sleep, and a target that moved (a REF stepped the clock
 * either way) is recomputed instead of spun on: the spin never lasts more than 2 x WAKE_EARLY. */
static void *pps_main(void *arg)
{
    (void)arg;
    int pin = g.px.pps_gpio;
    while (g_running) {
        pthread_mutex_lock(&g.core_mx);
        bool synced = tsync_is_synced(g.ts);
        uint64_t t = now_us();
        uint64_t gnow = tsync_global_us(g.ts, t);
        uint64_t edge_g = (gnow / 1000000u + 1u) * 1000000u;
        uint64_t edge_l = tsync_local_for_global(g.ts, edge_g);
        pthread_mutex_unlock(&g.core_mx);
        if (!synced) {                           /* no global time yet (follower before its first REF pair) */
            sleep_until_us(t + 100000u);
            continue;
        }
        if (edge_l > t + 1100000u || edge_l + PPS_LATE_LIMIT_US < t) {
            sleep_until_us(t + 1000u);           /* the model moved under us: recompute */
            continue;
        }
        if (edge_l > t + PPS_WAKE_EARLY_US)
            sleep_until_us(edge_l - PPS_WAKE_EARLY_US);

        pthread_mutex_lock(&g.core_mx);          /* a REF may have corrected the model meanwhile */
        edge_l = tsync_local_for_global(g.ts, edge_g);
        pthread_mutex_unlock(&g.core_mx);
        t = now_us();
        if (edge_l > t + 2u * PPS_WAKE_EARLY_US || edge_l + PPS_LATE_LIMIT_US < t)
            continue;                            /* stepped while we slept: recompute, never spin long */
        while ((t = now_us()) < edge_l) {
            /* spin: at most 2 x PPS_WAKE_EARLY_US */
        }
        if (g.pps_pin_ok) gpio_write(pin, 1);

        int32_t drift = 0, corr = 0;
        pthread_mutex_lock(&g.core_mx);
        uint64_t gl = tsync_global_us(g.ts, t);
        tsync_status(g.ts, &drift, &corr);
        uint8_t leader = raft_leader(g.raft);
        pthread_mutex_unlock(&g.core_mx);
        node_log("PPS global=%llu.%06llu s local=%llu.%06llu s late=%lld us drift=%d ppb corr=%d us (%s)",
                 (unsigned long long)(gl / 1000000u), (unsigned long long)(gl % 1000000u),
                 (unsigned long long)(t / 1000000u), (unsigned long long)(t % 1000000u),
                 (long long)((int64_t)gl - (int64_t)edge_g), drift, corr,
                 leader == g.self ? "time master" : "follower");

        sleep_until_us(t + PPS_HIGH_US);
        if (g.pps_pin_ok) gpio_write(pin, 0);
    }
    if (g.pps_pin_ok) gpio_write(pin, 0);
    return NULL;
}

/* ------------------------------------------------------------------------------------------
 * Start / stop
 * ------------------------------------------------------------------------------------------ */
static int open_bus(int b)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { node_log("[can] socket: %s", strerror(errno)); return -1; }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    snprintf(ifr.ifr_name, sizeof ifr.ifr_name, "%s", IFACE[b]);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        node_log("[can] %s not found: run 'sudo sh setup_can.sh' and check the module wiring", IFACE[b]);
        close(s);
        return -1;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    int one = 1;
    struct timeval tv = { 0, 200000 };
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0 ||
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &one, sizeof one) < 0 ||
        setsockopt(s, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof one) < 0 ||
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv) < 0) {
        node_log("[can] %s setup: %s", IFACE[b], strerror(errno));
        close(s);
        return -1;
    }
    return s;
}

static pthread_t *started[NBUS + 3];

static int start_thread(pthread_t *th, void *(*fn)(void *), void *arg, int prio)
{
    if (prio > 0) {
        pthread_attr_t a;
        pthread_attr_init(&a);
        pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&a, SCHED_FIFO);
        struct sched_param sp = { .sched_priority = prio };
        pthread_attr_setschedparam(&a, &sp);
        if (g.px.cpu >= 0) {
            cpu_set_t cs;
            CPU_ZERO(&cs);
            CPU_SET(g.px.cpu, &cs);
            pthread_attr_setaffinity_np(&a, sizeof cs, &cs);
        }
        int rc = pthread_create(th, &a, fn, arg);
        pthread_attr_destroy(&a);
        if (rc == 0) goto ok;
        node_log("[rt] SCHED_FIFO %d unavailable (%s): run as root; continuing without it", prio, strerror(rc));
    }
    if (pthread_create(th, NULL, fn, arg) != 0) return -1;
ok:
    started[g.n_threads_started++] = th;
    return 0;
}

int pi_node_start(const char *config_json_path)
{
    if (g_running) return -1;
    char err[160];
    memset(&g, 0, sizeof g);
    g.sock[0] = g.sock[1] = -1;
    if (config_load(config_json_path ? config_json_path : "raft_config.json", &g.rc, &g.px, err, sizeof err)) {
        fprintf(stderr, "[pi_node] %s\n", err);
        return -1;
    }
    g.self = g.rc.self_id;

    pthread_mutex_init(&g.core_mx, NULL);
    pthread_mutex_init(&g.st_mx, NULL);
    pthread_mutex_init(&g.tx_mx, NULL);
    if (ring_init(&g.rxq, sizeof(rx_item_t), 256) || ring_init(&g.appq, sizeof(app_item_t), 256) ||
        ring_init(&g.logq, sizeof(log_item_t), 1024) || ring_init(&g.msgq, sizeof(msg_item_t), 256))
        return -1;

    g.raft = calloc(1, sizeof(raft_t));
    g.logmem = calloc(1, RAFT_LOG_BYTES(g.rc.log_size));
    g.ts = calloc(1, sizeof(tsync_t));
    if (!g.raft || !g.logmem || !g.ts) return -1;

    g_running = 1;
    if (start_thread(&g.th_log, log_main, NULL, 0)) { g_running = 0; return -1; }

    for (int b = 0; b < NBUS; b++) {
        g.sock[b] = open_bus(b);
        if (g.sock[b] < 0) { pi_node_stop(); return -1; }
    }

    uint64_t t0 = now_us();
    g.last_rx[0] = g.last_rx[1] = t0;
    g.own_links = 0x3;
    g.cockpit_ok = false;                  /* Lab 3: power up in error until cockpit.py says ok */
    g.err_bits = ERR_CTRL_LOST;
    g.boot_us = t0;
    ts_prev_leader = 0;
    raft_init(g.raft, &g.rc, g.logmem, t0, on_apply, on_debug, NULL);
    tsync_init(g.ts, g.self, g.px.rx_latency_us, g.px.relay_delay_us);
    tsync_set_period_ms(g.ts, g.rc.heartbeat_ms);

    node_log("[pi_node] catalog v%s, self %s, default leader %s, heartbeat %u ms, log %u entries, seed %u%s",
             RAFT_CATALOG_VERSION, raft_node_name(g.self),
             g.rc.default_leader ? raft_node_name(g.rc.default_leader) : "none (election)",
             g.rc.heartbeat_ms, g.rc.log_size, (unsigned)g.rc.election_seed, g.rc.debug ? ", debug" : "");

    int prio = g.px.rt_priority;
    for (int b = 0; b < NBUS; b++)
        if (start_thread(&g.th_rx[b], rx_main, (void *)(intptr_t)b, prio)) { pi_node_stop(); return -1; }
    if (start_thread(&g.th_tick, tick_main, NULL, prio > 1 ? prio - 1 : prio)) { pi_node_stop(); return -1; }

    g.pps_pin_ok = false;
    if (g.px.pps_gpio >= 0) {
        if (gpio_init() == 0) {
            gpio_output(g.px.pps_gpio);
            gpio_write(g.px.pps_gpio, 0);
            g.pps_pin_ok = true;
            node_log("[pps] on GPIO %d (BCM)", g.px.pps_gpio);
        } else {
            node_log("[pps] /dev/gpiomem unavailable (add the user to the gpio group): PPS lines only, no pin");
        }
    }
    if (start_thread(&g.th_pps, pps_main, NULL, prio)) { pi_node_stop(); return -1; }
    return 0;
}

void pi_node_stop(void)
{
    if (!g_running) return;
    g_running = 0;
    for (int i = g.n_threads_started - 1; i >= 0; i--)
        pthread_join(*started[i], NULL);
    g.n_threads_started = 0;
    for (int b = 0; b < NBUS; b++)
        if (g.sock[b] >= 0) { close(g.sock[b]); g.sock[b] = -1; }
    free(g.raft);
    free(g.logmem);
    free(g.ts);
    g.raft = NULL;
    g.logmem = NULL;
    g.ts = NULL;
    ring_free(&g.rxq);
    ring_free(&g.appq);
    ring_free(&g.logq);
    ring_free(&g.msgq);
}

/* ------------------------------------------------------------------------------------------
 * API for cockpit.py
 * ------------------------------------------------------------------------------------------ */
int pi_node_send_app(uint16_t id, const uint8_t data[8])
{
    if (!g_running) return -1;
    uint8_t m = own_links();                 /* Lab 3: non-heartbeat traffic only on healthy buses */
    if (m) send_frame(m, id, data);
    return 0;
}

int pi_node_poll_app(uint16_t *id, uint8_t data[8], uint8_t *bus, int timeout_ms)
{
    if (!g_running) return -1;
    app_item_t a;
    int rc = ring_pop(&g.appq, &a, timeout_ms);
    if (rc) return rc;
    if (id) *id = a.id;
    if (data) memcpy(data, a.d, 8);
    if (bus) *bus = a.bus;
    return 0;
}

void pi_node_set_cockpit_state(bool ok, uint8_t err_bits)
{
    pthread_mutex_lock(&g.st_mx);
    if (ok != g.cockpit_ok || err_bits != g.err_bits)
        node_log("[cockpit] %s reasons=0x%02x", ok ? "NORMAL" : "ERROR", err_bits);
    g.cockpit_ok = ok;
    g.err_bits = err_bits;
    pthread_mutex_unlock(&g.st_mx);
}

int pi_node_report_event(uint8_t code)
{
    if (!g_running) return -1;
    pthread_mutex_lock(&g.core_mx);
    bool ok = raft_report(g.raft, RAFT_ENT_MAKE_EVENT(g.self, code & 7));
    pthread_mutex_unlock(&g.core_mx);
    return ok ? 0 : -1;
}

void pi_node_status(raft_net_t *net, uint8_t *leader, uint8_t *term, bool *failure,
                    uint8_t *bus_ok_mask, int32_t *drift_ppb)
{
    if (!g_running) return;
    int32_t drift = 0, corr = 0;
    pthread_mutex_lock(&g.core_mx);
    if (net) *net = raft_get_network(g.raft);
    if (leader) *leader = raft_leader(g.raft);
    if (term) *term = raft_term(g.raft);
    if (failure) *failure = raft_failure_mode(g.raft);
    tsync_status(g.ts, &drift, &corr);
    pthread_mutex_unlock(&g.core_mx);
    if (drift_ppb) *drift_ppb = drift;
    if (bus_ok_mask) *bus_ok_mask = own_links();
}

int pi_node_poll_log(uint16_t *index, uint8_t *term, uint8_t *entry, bool *replayed, int timeout_ms)
{
    if (!g_running) return -1;
    log_item_t li;
    int rc = ring_pop(&g.logq, &li, timeout_ms);
    if (rc) return rc;
    if (index) *index = li.index;
    if (term) *term = li.term;
    if (entry) *entry = li.entry;
    if (replayed) *replayed = li.replayed;
    return 0;
}
