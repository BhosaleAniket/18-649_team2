#include "tsync.h"

#include <string.h>

#define STEP_LIMIT_US   1000
#define PPB_LIMIT       500000          /* 500 ppm */
#define MIN_SPAN_SAMPLES 10
#define OUTLIER_US      300
#define SETTLED_SLEWS   10
#define OUTLIER_RUN     2       /* skip up to 2 in a row; the 3rd is real */

static int64_t slew_part(const tsync_t *t, int64_t d)
{
    if (t->slew_e == 0 || d <= 0 || t->slew_p == 0) {
        return 0;
    }
    if (d >= (int64_t)t->slew_p) {
        return t->slew_e;
    }
    return (int64_t)t->slew_e * d / (int64_t)t->slew_p;
}

static int64_t model(const tsync_t *t, uint64_t local)
{
    if (!t->synced) {
        return (int64_t)local;
    }
    int64_t d = (int64_t)(local - t->a_local);
    return t->a_global + d + d * t->ppb / 1000000000LL + slew_part(t, d);
}

void tsync_init(tsync_t *t, uint8_t self_id, uint32_t rx_latency_us, uint32_t relay_delay_us)
{
    memset(t, 0, sizeof(*t));
    t->self = self_id;
    t->rx_latency_us = rx_latency_us;
    t->relay_delay_us = relay_delay_us;
    t->period_us = 20000;
}

void tsync_set_period_ms(tsync_t *t, uint16_t heartbeat_ms)
{
    t->period_us = (uint32_t)heartbeat_ms * 1000u;
}

uint64_t tsync_global_us(const tsync_t *t, uint64_t now_local_us)
{
    int64_t g = model(t, now_local_us);
    return g < 0 ? 0 : (uint64_t)g;
}

uint64_t tsync_local_for_global(const tsync_t *t, uint64_t global_us)
{
    if (!t->synced) {
        return global_us;
    }
    int64_t l = (int64_t)t->a_local + ((int64_t)global_us - t->a_global);
    for (int i = 0; i < 4; i++) {
        int64_t err = model(t, (uint64_t)l) - (int64_t)global_us;
        if (err == 0) {
            break;
        }
        l -= err;
    }
    return l < 0 ? 0 : (uint64_t)l;             /* an edge from before a forward step: in the past, never a wrapped value */
}

bool tsync_is_synced(const tsync_t *t)
{
    return t->synced;
}

void tsync_status(const tsync_t *t, int32_t *drift_ppb, int32_t *last_corr_us)
{
    if (drift_ppb) {
        *drift_ppb = t->ppb;
    }
    if (last_corr_us) {
        *last_corr_us = t->last_corr;
    }
}

void tsync_on_leader_change(tsync_t *t)
{
    t->paired_valid = false;
    memset(t->rx, 0, sizeof(t->rx));
    t->ns = 0;
    t->head = 0;
}

/* Re-anchor at `local` keeping the global time continuous (folds the slew in) */
static void reanchor(tsync_t *t, uint64_t local)
{
    t->a_global = model(t, local);
    t->a_local = local;
    t->slew_e = 0;
}

bool tsync_on_tick(tsync_t *t, bool leader, uint8_t term, uint64_t now, raft_frame_t *ref)
{
    if (!leader) {
        t->was_leader = false;
        return false;
    }
    if (!t->was_leader) {
        t->was_leader = true;
        if (!t->synced) {
            t->synced = true;                   /* cold start: global time = this node's clock */
            t->a_local = now;
            t->a_global = (int64_t)now;
        } else {
            reanchor(t, now);                   /* keep running at the last drift estimate */
        }
        t->next_ref = now;                      /* first REF at once */
        t->stamp_valid[0] = t->stamp_valid[1] = false;
        t->inflight_valid[0] = t->inflight_valid[1] = false;
    }
    if (now < t->next_ref) {
        return false;
    }

    memset(ref, 0, sizeof(*ref));
    ref->id = (uint16_t)(RAFT_ID_REF + t->self);
    ref->bus_mask = 0x03;
    ref->data[1] = term;
    ref->data[2] = t->cycle;
    /* Stamp bus alternates (A after even cycles, B after odd, else the
     * other one), so a follower that is only on one bus still gets a
     * stamp it can pair on every other REF. */
    uint8_t prev = (uint8_t)(t->cycle - 1);
    int first = prev & 1;
    for (int k = 0; k < 2; k++) {
        int b = k == 0 ? first : 1 - first;
        if (t->stamp_valid[b] && t->stamp_cycle[b] == prev) {
            uint64_t s = t->stamp[b];
            ref->data[3] = (uint8_t)((b + 1) | (((s >> 32) & 0x3F) << 2));
            raft_put32(&ref->data[4], (uint32_t)s);
            break;
        }
    }
    for (int b = 0; b < 2; b++) {
        t->inflight_valid[b] = true;
        t->inflight_cycle[b] = t->cycle;
    }
    t->cycle++;
    t->next_ref += t->period_us;
    if (t->next_ref <= now) {
        t->next_ref = now + t->period_us;
    }
    return true;
}

void tsync_on_tx_done(tsync_t *t, uint8_t bus, uint64_t tx_local)
{
    if (bus < 1 || bus > 2 || !t->inflight_valid[bus - 1]) {
        return;
    }
    int b = bus - 1;
    t->inflight_valid[b] = false;
    t->stamp[b] = tsync_global_us(t, tx_local);
    t->stamp_cycle[b] = t->inflight_cycle[b];
    t->stamp_valid[b] = true;
}

/* A buffered arrival of REF `cycle` on bus b. Only arrivals from the last
 * 3 REF periods count: the cycle is 8 bits and wraps every 256 REFs
 * (5.12 s), so an entry left over on a bus that went quiet would
 * otherwise match again and pair a stamp with a stale receive time. */
static bool find_rx(const tsync_t *t, int b, uint8_t cycle, bool want_relayed, uint64_t now,
                    uint64_t *local)
{
    uint64_t window = 3u * (uint64_t)t->period_us;
    for (int i = 0; i < 4; i++) {
        if (t->rx[b][i].used && t->rx[b][i].cycle == cycle && t->rx[b][i].relayed == want_relayed &&
            t->rx[b][i].local <= now && now - t->rx[b][i].local <= window) {
            *local = t->rx[b][i].local;
            return true;
        }
    }
    return false;
}

static void update_drift(tsync_t *t, uint64_t f, int64_t g)
{
    t->samp[t->head].f = f;
    t->samp[t->head].g = g;
    t->head = (t->head + 1) % TSYNC_SAMPLES;
    if (t->ns < TSYNC_SAMPLES) {
        t->ns++;
    }
    if (t->ns < MIN_SPAN_SAMPLES) {
        return;
    }
    int span = t->ns - 1 < TSYNC_DRIFT_SPAN ? t->ns - 1 : TSYNC_DRIFT_SPAN;
    int old = (t->head - 1 - span + 2 * TSYNC_SAMPLES) % TSYNC_SAMPLES;
    int64_t df = (int64_t)(f - t->samp[old].f);
    int64_t dg = g - t->samp[old].g;
    if (df < 100000) {                          /* need at least 100 ms of baseline */
        return;
    }
    int64_t meas = (dg - df) * 1000000000LL / df;
    if (meas > PPB_LIMIT) meas = PPB_LIMIT;
    if (meas < -PPB_LIMIT) meas = -PPB_LIMIT;
    if (t->ns < TSYNC_DRIFT_SPAN) {
        t->ppb = (int32_t)meas;                 /* early on: take the measurement */
    } else {
        t->ppb += (int32_t)((meas - t->ppb) / 10);  /* alpha = 0.1 */
    }
}

void tsync_on_ref(tsync_t *t, uint8_t bus, const uint8_t d[8], bool relayed, uint64_t rx_local)
{
    if (bus < 1 || bus > 2) {
        return;
    }
    int b = bus - 1;
    uint8_t cycle = d[2];

    /* Remember when this REF arrived on this bus */
    int i = t->rx_head[b];
    t->rx[b][i].used = true;
    t->rx[b][i].cycle = cycle;
    t->rx[b][i].relayed = relayed;
    t->rx[b][i].local = rx_local;
    t->rx_head[b] = (uint8_t)((i + 1) % 4);

    uint8_t sbus = d[3] & 0x03;
    if (sbus < 1 || sbus > 2) {
        return;                                 /* no stamp yet (first REF of a leader) */
    }
    /* Both copies of a REF carry the same stamp: use it once. Later copies
     * only record their arrival (above), which a later stamp may need. */
    if (t->paired_valid && t->paired_cycle == cycle) {
        return;
    }
    t->paired_valid = true;
    t->paired_cycle = cycle;
    uint8_t prev = (uint8_t)(cycle - 1);
    uint64_t f;
    uint32_t extra = t->rx_latency_us;
    if (!find_rx(t, sbus - 1, prev, false, rx_local, &f)) {
        /* No direct copy on the stamp bus: use a relayed copy if there is one */
        if (find_rx(t, 0, prev, true, rx_local, &f) || find_rx(t, 1, prev, true, rx_local, &f)) {
            extra += t->relay_delay_us;
        } else {
            return;
        }
    }
    uint64_t stamp = ((uint64_t)(d[3] >> 2) << 32) | raft_get32(&d[4]);
    int64_t g = (int64_t)(stamp + extra);
    int64_t pred = model(t, f);
    /* The error is measured at f, one REF ago. Part of an earlier correction
     * may have been slewed in since f; take it off, or it is applied twice
     * and the clock oscillates. */
    int64_t already = 0;
    if (t->synced && t->slew_e != 0) {
        int64_t d_now = (int64_t)(rx_local - t->a_local);
        int64_t d_f = (int64_t)(f - t->a_local);
        already = slew_part(t, d_now) - slew_part(t, d_f > 0 ? d_f : 0);
    }
    int64_t e = g - pred - already;

    bool big = e > OUTLIER_US || e < -OUTLIER_US;
    if (t->synced && t->slews >= SETTLED_SLEWS && big && t->outlier_run < OUTLIER_RUN) {
        t->outlier_run++;                       /* a late stamp: ignore it */
        t->outliers++;
        return;
    }
    t->outlier_run = 0;

    if (!t->synced || e > STEP_LIMIT_US || e < -STEP_LIMIT_US) {
        t->synced = true;
        t->a_local = f;
        t->a_global = g;
        t->slew_e = 0;
        t->steps++;
    } else {
        /* The error was measured at f (one REF ago); slew it in from now,
         * so values already handed out (f .. now) never jump. */
        reanchor(t, rx_local);
        t->slew_e = (int32_t)e;
        t->slew_p = t->period_us;
        t->slews++;
    }
    t->last_corr = (int32_t)e;
    update_drift(t, f, g);
}
