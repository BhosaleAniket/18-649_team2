/*
 * sim.c - three nodes, two CAN buses, simulated time. Runs the shared
 * raft_core / route / tsync code against the Lab 5 checkoff scenarios.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "raft_core.h"
#include "route.h"
#include "tsync.h"

#define LOG_SIZE 30000
#define FRAME_US 250
#define RELAY_US 300
#define MAXEV 20000

static int verbose = 0;
static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("    FAIL: "); printf(__VA_ARGS__); printf("\n"); } else { printf("    ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct {
    raft_t r;
    tsync_t ts;
    uint8_t logmem[RAFT_LOG_BYTES(LOG_SIZE)];
    raft_config_t cfg;
    bool powered, silent;
    bool wire[2];                   /* jumper in, per bus */
    double drift_ppm;
    double offset_us;
    uint8_t prev_leader;
    /* dedup */
    struct { uint16_t id; uint8_t key[8]; uint64_t t; } seen[64];
    int seen_head;
    /* stats */
    int applied_replayed, applied_total;
    uint64_t last_global;
    bool have_last_global;
    double max_step_us;
    int req_votes_sent;
} node_t;

static node_t N[4];
static uint64_t now;                /* true time, us */
static uint32_t seed_rng = 12345;
static int frames_on_bus[2];
static int relayed_frames;

static uint32_t srnd(void) { seed_rng = seed_rng * 1103515245u + 12345u; return seed_rng >> 8; }

static uint64_t local_of(int n, uint64_t t) { return (uint64_t)((double)t * (1.0 + N[n].drift_ppm * 1e-6) + N[n].offset_us); }

/* ---- event queue ---- */
typedef struct { uint64_t t; int kind; int node; int bus; raft_frame_t f; bool relayed; } ev_t;
enum { EV_RX = 1, EV_TXDONE = 2 };
static ev_t evq[MAXEV];
static int nev;

static void push_ev(ev_t e)
{
    if (nev >= MAXEV) { printf("event queue full\n"); exit(1); }
    evq[nev++] = e;
}

static bool on_bus(int n, int bus) { return N[n].powered && N[n].wire[bus - 1]; }

static void send_on(int from, int bus, const raft_frame_t *f, bool relayed, bool is_ref)
{
    if (!on_bus(from, bus)) return;
    frames_on_bus[bus - 1]++;
    uint64_t done = now + FRAME_US;
    for (int m = 1; m <= 3; m++) {
        if (m == from || !on_bus(m, bus)) continue;
        uint64_t late = (is_ref && getenv("LATE") && srnd() % 50 == 0) ? 400 : 0;
        ev_t e = { done + (srnd() % 20) + late, EV_RX, m, bus, *f, relayed };
        push_ev(e);
    }
    if (is_ref && !relayed) {
        ev_t e = { done + (srnd() % 20), EV_TXDONE, from, bus, *f, false };
        push_ev(e);
    }
}

static void send_frame(int from, const raft_frame_t *f)
{
    bool is_ref = raft_id_base(f->id) == RAFT_ID_REF;
    if (raft_id_base(f->id) == RAFT_ID_REQ_VOTE && f->data[5] == 0) N[from].req_votes_sent++;
    for (int bus = 1; bus <= 2; bus++) {
        if (f->bus_mask & RAFT_BUSBIT(bus)) send_on(from, bus, f, false, is_ref);
    }
}

static void flush_out(int n, raft_out_t *out)
{
    for (int i = 0; i < out->n; i++) send_frame(n, &out->f[i]);
    out->n = 0;
}

/* heard[]: own detection for self; others from their heartbeat (wire state, 0 if silent/off) */
static bool hears(int m, int n)
{
    /* can m hear n's heartbeat directly or via one relay? */
    if (!N[n].powered || N[n].silent) return false;
    for (int b = 1; b <= 2; b++) if (on_bus(m, b) && on_bus(n, b)) return true;
    for (int c = 1; c <= 3; c++) {
        if (c == m || c == n || !N[c].powered) continue;
        bool cm = false, cn = false;
        for (int b = 1; b <= 2; b++) { cm |= on_bus(c, b) && on_bus(m, b); cn |= on_bus(c, b) && on_bus(n, b); }
        if (cm && cn) return true;
    }
    return false;
}

static uint8_t own_mask(int n)
{
    /* DEG detection: a bus is up if anyone else is on it */
    uint8_t m = 0;
    for (int b = 1; b <= 2; b++) {
        if (!on_bus(n, b)) continue;
        for (int o = 1; o <= 3; o++) if (o != n && on_bus(o, b) && !N[o].silent) m |= RAFT_BUSBIT(b);
    }
    return m;
}

static void heard_for(int self, uint8_t heard[3])
{
    for (int n = 1; n <= 3; n++) heard[n - 1] = (n == self || hears(self, n)) ? own_mask(n) : 0;
}

static bool dedup(node_t *nd, const raft_frame_t *f)
{
    uint8_t key[8];
    memcpy(key, f->data, 8);
    key[0] &= 0x0F;
    for (int i = 0; i < 64; i++) {
        if (nd->seen[i].t && nd->seen[i].id == f->id && memcmp(nd->seen[i].key, key, 8) == 0 &&
            now - nd->seen[i].t < 150000) return true;
    }
    nd->seen[nd->seen_head].id = f->id;
    memcpy(nd->seen[nd->seen_head].key, key, 8);
    nd->seen[nd->seen_head].t = now;
    nd->seen_head = (nd->seen_head + 1) % 64;
    return false;
}

static void deliver(int n, int bus, raft_frame_t f, bool relayed)
{
    (void)relayed;
    node_t *nd = &N[n];
    if (!nd->powered) return;
    if (dedup(nd, &f)) return;
    uint64_t lt = local_of(n, now);
    uint8_t sender = raft_id_sender(f.id);
    raft_out_t out = { .n = 0 };

    /* relay first, as the CAN layer would */
    if (!nd->silent) {
        uint8_t heard[3];
        heard_for(n, heard);
        raft_net_t net = raft_get_network(&nd->r);
        uint8_t rm = route_relay_mask(&net, heard, (uint8_t)n, sender, (uint8_t)bus, (f.data[0] & RAFT_FLAG_RELAYED) != 0);
        if (rm) {
            raft_frame_t g = f;
            g.data[0] |= RAFT_FLAG_RELAYED;
            relayed_frames++;
            uint64_t save = now;
            now += RELAY_US;
            send_on(n, rm == RAFT_BUSBIT(1) ? 1 : 2, &g, true, false);
            now = save;
        }
    }

    raft_on_frame(&nd->r, (uint8_t)bus, f.id, f.data, lt, &out);
    if (raft_id_base(f.id) == RAFT_ID_REF && raft_leader(&nd->r) == sender && !raft_is_leader(&nd->r)) {
        tsync_on_ref(&nd->ts, (uint8_t)bus, f.data, (f.data[0] & RAFT_FLAG_RELAYED) != 0, lt);
    }
    if (!nd->silent) flush_out(n, &out);
}

static void apply_cb(void *ctx, uint16_t idx, uint8_t term, uint8_t e, bool replayed)
{
    node_t *nd = ctx;
    nd->applied_total++;
    if (replayed) nd->applied_replayed++;
    if (verbose) {
        char s[48];
        raft_entry_str(e, s, sizeof(s));
        printf("      [%7.3f s] N%d apply idx %u term %u %s%s\n", now / 1e6, nd->cfg.self_id, idx, term, s, replayed ? " (replayed)" : "");
    }
}

static void dbg_cb(void *ctx, const char *line)
{
    node_t *nd = ctx;
    if (verbose > 1) printf("      [%7.3f s] N%d %s\n", now / 1e6, nd->cfg.self_id, line);
}

static void boot(int n, uint8_t default_leader, uint32_t seed)
{
    node_t *nd = &N[n];
    memset(&nd->r, 0, sizeof(nd->r));
    nd->cfg = (raft_config_t){ 3, (uint8_t)n, default_leader, 20, 100, 100, 3, 39, 2, 50, seed, LOG_SIZE, true };
    raft_init(&nd->r, &nd->cfg, nd->logmem, local_of(n, now), apply_cb, dbg_cb, nd);
    tsync_init(&nd->ts, (uint8_t)n, 0, RELAY_US + FRAME_US);
    nd->powered = true;
    nd->silent = false;
    nd->prev_leader = 0;
    nd->have_last_global = false;
    nd->applied_replayed = nd->applied_total = 0;
    memset(nd->seen, 0, sizeof(nd->seen));
}

static void tick_node(int n)
{
    node_t *nd = &N[n];
    if (!nd->powered) return;
    uint64_t lt = local_of(n, now);
    raft_out_t out = { .n = 0 };

    raft_set_own_links(&nd->r, own_mask(n));
    if (now > 300000) {
        uint8_t alive = 0;
        for (int m = 1; m <= 3; m++) if (m != n && hears(n, m)) alive |= RAFT_NODEBIT(m);
        raft_observe_peers(&nd->r, alive);
        raft_observe_cockpit(&nd->r, n == 1 ? true : hears(n, 1));
    }
    raft_on_tick(&nd->r, lt, &out);
    flush_out(n, &out);

    uint8_t l = raft_leader(&nd->r);
    if (l != nd->prev_leader) {
        if (nd->prev_leader != 0) tsync_on_leader_change(&nd->ts);
        nd->prev_leader = l;
    }
    raft_frame_t ref;
    if (tsync_on_tick(&nd->ts, raft_is_leader(&nd->r), raft_term(&nd->r), lt, &ref)) send_frame(n, &ref);

    /* step watch: global time should advance by ~1 ms per ms */
    if (tsync_is_synced(&nd->ts) && !nd->silent) {
        uint64_t g = tsync_global_us(&nd->ts, lt);
        if (nd->have_last_global) {
            double step = fabs((double)(int64_t)(g - nd->last_global) - 1000.0);
            if (step > nd->max_step_us) nd->max_step_us = step;
        }
        nd->last_global = g;
        nd->have_last_global = true;
    } else {
        nd->have_last_global = false;
    }
}

static int cmp_ev(const void *a, const void *b)
{
    const ev_t *x = a, *y = b;
    return x->t < y->t ? -1 : x->t > y->t;
}

static double spread_max;
static bool measure_spread;

static void run_until(uint64_t t_end)
{
    while (now < t_end) {
        uint64_t next_tick = (now / 1000 + 1) * 1000;
        qsort(evq, nev, sizeof(ev_t), cmp_ev);
        while (nev > 0 && evq[0].t < next_tick) {
            ev_t e = evq[0];
            memmove(evq, evq + 1, (size_t)(nev - 1) * sizeof(ev_t));
            nev--;
            now = e.t;
            if (e.kind == EV_RX) deliver(e.node, e.bus, e.f, e.relayed);
            else if (e.kind == EV_TXDONE && N[e.node].powered) tsync_on_tx_done(&N[e.node].ts, (uint8_t)e.bus, local_of(e.node, now));
            qsort(evq, nev, sizeof(ev_t), cmp_ev);
        }
        now = next_tick;
        for (int n = 1; n <= 3; n++) tick_node(n);
        if (measure_spread) {
            double lo = 1e18, hi = -1e18;
            int cnt = 0;
            for (int n = 1; n <= 3; n++) {
                if (!N[n].powered || N[n].silent || !tsync_is_synced(&N[n].ts)) continue;
                double g = (double)tsync_global_us(&N[n].ts, local_of(n, now));
                if (g < lo) lo = g;
                if (g > hi) hi = g;
                cnt++;
            }
            if (cnt >= 2 && hi - lo > spread_max) spread_max = hi - lo;
        }
    }
}

/* ---- checks ---- */
static int leaders(uint8_t *who, uint8_t *term)
{
    int c = 0;
    for (int n = 1; n <= 3; n++) if (N[n].powered && raft_is_leader(&N[n].r)) { c++; *who = (uint8_t)n; *term = raft_term(&N[n].r); }
    return c;
}

static bool logs_agree(int *common)
{
    int c = 65535;
    for (int n = 1; n <= 3; n++) if (N[n].powered && !N[n].silent) { int k = raft_commit_index(&N[n].r); if (k < c) c = k; }
    for (int i = 1; i <= c; i++) {
        uint8_t t0 = 0, e0 = 0; bool first = true;
        for (int n = 1; n <= 3; n++) {
            if (!N[n].powered || N[n].silent) continue;
            uint8_t t, e;
            if (!raft_log_entry(&N[n].r, (uint16_t)i, &t, &e)) return false;
            if (first) { t0 = t; e0 = e; first = false; }
            else if (t != t0 || e != e0) { *common = i; return false; }
        }
    }
    *common = c;
    return true;
}

static bool nets_agree(void)
{
    raft_net_t a; bool have = false;
    for (int n = 1; n <= 3; n++) {
        if (!N[n].powered || N[n].silent) continue;
        raft_net_t b = raft_get_network(&N[n].r);
        if (!have) { a = b; have = true; }
        else if (memcmp(&a, &b, sizeof(a)) != 0) return false;
    }
    return true;
}

static void print_net(int n)
{
    raft_net_t x = raft_get_network(&N[n].r);
    printf("      N%d getNetwork: alive=%d%d%d links cockpit=%c%c steering=%c%c drivetrain=%c%c failure=%d commit=%u term=%u leader=%u\n",
           n, x.alive & 1, (x.alive >> 1) & 1, (x.alive >> 2) & 1,
           (x.link[0] & 1) ? 'A' : '-', (x.link[0] & 2) ? 'B' : '-',
           (x.link[1] & 1) ? 'A' : '-', (x.link[1] & 2) ? 'B' : '-',
           (x.link[2] & 1) ? 'A' : '-', (x.link[2] & 2) ? 'B' : '-',
           raft_failure_mode(&N[n].r), raft_commit_index(&N[n].r), raft_term(&N[n].r), raft_leader(&N[n].r));
}

static void reset_world(void)
{
    memset(N, 0, sizeof(N));
    nev = 0;
    now = 0;
    relayed_frames = 0;
    frames_on_bus[0] = frames_on_bus[1] = 0;
    spread_max = 0;
    measure_spread = false;
    double drifts[4] = { 0, 35.0, -42.0, 18.0 };
    double offs[4] = { 0, 5e6, 123456.0, 777777.0 };
    for (int n = 1; n <= 3; n++) {
        N[n].wire[0] = N[n].wire[1] = true;
        N[n].drift_ppm = drifts[n];
        N[n].offset_us = offs[n];
    }
}

static void press(int n)
{
    N[n].silent = true;
    raft_set_silent(&N[n].r, true);
    for (int m = 1; m <= 3; m++) if (m != n && N[m].powered) raft_note_test_fault(&N[m].r, (uint8_t)n);
}

static int count_entries(int n, uint8_t entry)
{
    int c = 0;
    for (uint16_t i = 1; i <= raft_commit_index(&N[n].r); i++) {
        uint8_t t, e;
        raft_log_entry(&N[n].r, i, &t, &e);
        if (e == entry) c++;
    }
    return c;
}

static void reset_steps(void) { for (int n = 1; n <= 3; n++) N[n].max_step_us = 0; }

int main(int argc, char **argv)
{
    if (argc > 1) verbose = atoi(argv[1]);
    uint8_t who = 0, term = 0;
    int common;

    /* ---------------- 1. cold start, LEADER=cockpit, Pi boots 2 s late ---------------- */
    printf("1. Cold start, LEADER=cockpit (Pi boots 2 s after the Nucleos)\n");
    reset_world();
    boot(2, 1, 0); boot(3, 1, 0);
    N[1].powered = false;
    run_until(2000000);
    CHECK(leaders(&who, &term) == 0 && N[2].req_votes_sent + N[3].req_votes_sent == 0, "Nucleos wait for the Pi, no REQ_VOTE before it boots");
    boot(1, 1, 0);
    run_until(5000000);
    CHECK(leaders(&who, &term) == 1 && who == 1 && term == 1, "leader = cockpit, term 1 (got N%u term %u)", who, term);
    int rv = N[1].req_votes_sent + N[2].req_votes_sent + N[3].req_votes_sent;
    CHECK(rv == 0, "no REQ_VOTE (%d)", rv);
    measure_spread = true; spread_max = 0;
    frames_on_bus[0] = frames_on_bus[1] = 0;
    run_until(8000000);
    printf("      steady state: %.1f Raft frames/s on bus A, %.1f on bus B\n", frames_on_bus[0] / 3.0, frames_on_bus[1] / 3.0);
    CHECK(spread_max < 200, "global time spread across nodes %.0f us (< 200)", spread_max);
    CHECK(logs_agree(&common) && nets_agree(), "logs and graphs agree (commit %d)", common);
    print_net(2);

    /* ---------------- 2/4. link faults, no election ---------------- */
    printf("4. Link faults: pull drivetrain's bus A, then put it back\n");
    int c0 = raft_commit_index(&N[1].r);
    N[3].wire[0] = false;
    run_until(9000000);
    CHECK(leaders(&who, &term) == 1 && who == 1 && term == 1, "no election, term 1");
    CHECK(count_entries(1, RAFT_ENT_MAKE_LINK(3, 1, false)) == 1 && logs_agree(&common) && nets_agree(),
          "one LINK drivetrain A DOWN, same index everywhere (commit %d -> %d)", c0, common);
    print_net(3);
    N[3].wire[0] = true;
    run_until(10000000);
    CHECK(count_entries(2, RAFT_ENT_MAKE_LINK(3, 1, true)) == 1 && nets_agree(), "LINK drivetrain A UP committed");

    /* ---------------- 5. the hop: N1 off B, N3 off A ---------------- */
    printf("5. The hop with the Pi as master: cockpit off B, drivetrain off A\n");
    measure_spread = true; spread_max = 0;
    N[1].wire[1] = false;
    run_until(10500000);
    N[3].wire[0] = false;
    run_until(12000000);
    CHECK(leaders(&who, &term) == 1 && who == 1 && term == 1, "term unchanged (%u)", term);
    CHECK(count_entries(1, RAFT_ENT_MAKE_LINK(1, 2, false)) == 1 && count_entries(1, RAFT_ENT_MAKE_LINK(3, 1, false)) == 2,
          "both LINK DOWN entries committed");
    raft_report(&N[2].r, RAFT_ENT_MAKE_EVENT(2, 3));
    run_until(13000000);
    CHECK(count_entries(3, RAFT_ENT_MAKE_EVENT(2, 3)) == 1, "EVENT from steering reaches drivetrain's log once");
    CHECK(spread_max < 1000, "sync through the relay: spread %.0f us (< 1000)", spread_max);
    CHECK(logs_agree(&common) && nets_agree(), "logs and graphs agree");
    print_net(3);
    printf("      frames relayed so far: %d\n", relayed_frames);
    N[1].wire[1] = true; N[3].wire[0] = true;
    run_until(14000000);
    CHECK(nets_agree(), "links restored and agreed");

    /* ---------------- 6. Pi down ---------------- */
    printf("6. Pi down\n");
    reset_steps();
    uint64_t t_off = now;
    N[1].powered = false;
    uint64_t t_lead = 0;
    while (now < t_off + 1500000) {
        run_until(now + 1000);
        if (!t_lead && leaders(&who, &term) == 1 && who != 1) t_lead = now;
    }
    CHECK(t_lead && t_lead - t_off < 1000000, "STM32 leader N%u, term %u, %.0f ms after the Pi went off", who, term, (t_lead - t_off) / 1000.0);
    CHECK(term == 2, "term changed exactly once (%u)", term);
    double st = fmax(N[2].max_step_us, N[3].max_step_us);
    CHECK(st < 100, "largest global-time step at the handover %.1f us (< 100)", st);
    run_until(now + 1000000);
    CHECK(raft_failure_mode(&N[2].r) && count_entries(2, RAFT_ENT_MAKE_NODE(1, false)) >= 1, "COCKPIT_ERROR and NODE cockpit DOWN committed");
    raft_report(&N[2].r, RAFT_ENT_MAKE_EVENT(2, 5));
    raft_report(&N[3].r, RAFT_ENT_MAKE_EVENT(3, 6));
    run_until(now + 500000);
    uint8_t stm_leader = who;

    /* ---------------- 7. Pi back ---------------- */
    printf("7. Pi back (fresh boot, LEADER=cockpit)\n");
    boot(1, 1, 0);
    run_until(now + 3000000);
    CHECK(leaders(&who, &term) == 1 && who == stm_leader, "leader stays on the STM32 (N%u, term %u)", who, term);
    CHECK(raft_leader(&N[1].r) == stm_leader && !raft_is_leader(&N[1].r), "Pi rejoined as a follower");
    CHECK(N[1].applied_replayed > 0, "Pi applied %d entries marked replayed (of %d)", N[1].applied_replayed, N[1].applied_total);
    CHECK(count_entries(1, RAFT_ENT_MAKE_EVENT(2, 5)) == 1 && count_entries(1, RAFT_ENT_MAKE_EVENT(3, 6)) == 1, "held events on the Pi exactly once");
    CHECK(logs_agree(&common) && nets_agree() && !raft_failure_mode(&N[1].r), "logs agree, failure mode cleared (COCKPIT_OK)");
    print_net(1);

    /* ---------------- 9. relayed sync with an STM32 master ---------------- */
    printf("9. Relayed sync: STM32 master, the other STM32 cut off from it (Pi relays)\n");
    measure_spread = true; spread_max = 0;
    int other = stm_leader == 2 ? 3 : 2;
    N[stm_leader].wire[1] = false;      /* leader only on A */
    N[other].wire[0] = false;           /* other only on B */
    run_until(now + 3000000);
    CHECK(leaders(&who, &term) == 1 && who == stm_leader, "no election (term %u)", term);
    CHECK(spread_max < 1000, "spread %.0f us through the Pi relay (< 1000)", spread_max);
    reset_steps();
    N[stm_leader].wire[1] = true; N[other].wire[0] = true;
    run_until(now + 2000000);
    CHECK(N[other].max_step_us < 100, "no step when restored (%.1f us)", N[other].max_step_us);

    /* ---------------- 10. isolation and test button ---------------- */
    printf("10. Isolation and the test button\n");
    leaders(&who, &term);
    uint8_t t0 = term;
    int follower = (who == 2) ? 3 : 2;
    N[follower].wire[0] = N[follower].wire[1] = false;
    run_until(now + 2000000);
    N[follower].wire[0] = N[follower].wire[1] = true;
    run_until(now + 2000000);
    leaders(&who, &term);
    CHECK(term == t0, "isolated follower: term unchanged (%u)", term);
    uint8_t ld = who;
    N[ld].wire[0] = N[ld].wire[1] = false;
    run_until(now + 2000000);
    N[ld].wire[0] = N[ld].wire[1] = true;
    run_until(now + 2000000);
    leaders(&who, &term);
    CHECK(term == (uint8_t)(t0 + 1), "isolated leader: term changed exactly once (%u -> %u)", t0, term);
    CHECK(logs_agree(&common), "logs agree after reconnection");
    t0 = term; ld = who;
    press(ld);
    run_until(now + 1500000);
    leaders(&who, &term);
    CHECK(who != ld && term == (uint8_t)(t0 + 1), "leader's test button: new leader N%u, term %u -> %u", who, t0, term);
    run_until(now + 500000);
    CHECK(count_entries(who, RAFT_ENT_MAKE_EVENT(ld, RAFT_EVT_TEST_BUTTON)) == 1, "test-button press logged once as EVENT");
    N[ld].silent = false;
    raft_set_silent(&N[ld].r, false);
    run_until(now + 2000000);
    uint8_t t1 = term;
    leaders(&who, &term);
    CHECK(term == t1 && !raft_is_leader(&N[ld].r), "double press: rejoins as follower, term unchanged (%u)", term);
    CHECK(logs_agree(&common) && nets_agree(), "logs and graphs agree");

    /* ---------------- 8. random leader ---------------- */
    printf("8. LEADER=random: seeds that make each node win\n");
    int winner_for_seed[64];
    int pi_seed = -1;
    for (uint32_t s = 0; s < 64; s++) {
        reset_world();
        boot(1, 0, s); boot(2, 0, s); boot(3, 0, s);
        run_until(1500000);
        winner_for_seed[s] = leaders(&who, &term) == 1 ? who : 0;
        if (winner_for_seed[s] == 1 && pi_seed < 0) pi_seed = (int)s;
    }
    int ok_all = 1;
    for (int s = 0; s < 64; s++) if (winner_for_seed[s] == 0) ok_all = 0;
    CHECK(ok_all, "every seed elects exactly one leader within 1.5 s");
    CHECK(pi_seed >= 0, "election_seed %d makes the Pi win", pi_seed);
    reset_world();
    boot(1, 0, 3); boot(2, 0, 3); boot(3, 0, 3);
    run_until(2000000);
    leaders(&who, &term);
    uint8_t l0 = who, tt = term;
    N[l0].powered = false;
    run_until(now + 1000000);
    CHECK(leaders(&who, &term) == 1 && who != l0 && term == (uint8_t)(tt + 1), "leader off: new leader N%u within 1 s, term %u", who, term);

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
