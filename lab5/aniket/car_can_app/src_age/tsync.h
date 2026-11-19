/*
 * tsync.h - time sync over REF (catalog v0.8). Pure: no OS calls.
 *
 * Global time is a 64-bit microsecond count mastered by the Raft leader.
 * The leader stamps each REF's TX-complete time (in global time) and sends
 * it in the NEXT REF; a follower pairs it with its own RX time of that
 * same REF on the same bus. Both stamps are taken at end of frame.
 *
 *   global(local) = Ga + d + d * ppb / 1e9 + slew(d),  d = local - Fa
 * A correction e <= 1 ms is slewed in over one REF period (no step);
 * a larger one (first sync, long gap) steps. Once settled, a single
 * correction above 300 us is treated as an outlier (a REF timestamped late,
 * e.g. while an edge thread was spinning) and skipped; two in a row are real. ppb comes from samples about
 * 1 s apart (50 REFs), smoothed, clamped to +-500 ppm.
 */
#ifndef TSYNC_H
#define TSYNC_H

#include <stdbool.h>
#include <stdint.h>
#include "raft_core.h"

#define TSYNC_SAMPLES 52
#define TSYNC_DRIFT_SPAN 50

typedef struct tsync {
    uint8_t  self;
    uint32_t rx_latency_us, relay_delay_us, period_us;

    /* clock model */
    bool     synced;
    uint64_t a_local;
    int64_t  a_global;
    int32_t  ppb;
    int32_t  slew_e;            /* us still being slewed in over slew_p */
    uint32_t slew_p;
    int32_t  last_corr;
    uint32_t steps, slews, outliers;
    bool     prev_outlier;

    /* leader */
    bool     was_leader;
    uint64_t next_ref;
    uint8_t  cycle;
    bool     inflight_valid[2];
    uint8_t  inflight_cycle[2];
    bool     stamp_valid[2];
    uint8_t  stamp_cycle[2];
    uint64_t stamp[2];

    /* follower */
    struct { uint8_t cycle; bool relayed; bool used; uint64_t local; } rx[2][4];
    uint8_t  rx_head[2];
    struct { uint64_t f; int64_t g; } samp[TSYNC_SAMPLES];
    int      ns, head;
} tsync_t;

void     tsync_init(tsync_t *t, uint8_t self_id, uint32_t rx_latency_us, uint32_t relay_delay_us);
void     tsync_set_period_ms(tsync_t *t, uint16_t heartbeat_ms);
bool     tsync_on_tick(tsync_t *t, bool leader, uint8_t term, uint64_t now_local_us,
                       raft_frame_t *ref_out);                      /* true = send ref_out */
void     tsync_on_tx_done(tsync_t *t, uint8_t bus, uint64_t tx_local_us);
void     tsync_on_ref(tsync_t *t, uint8_t bus, const uint8_t data[8], bool relayed, uint64_t rx_local_us);
void     tsync_on_leader_change(tsync_t *t);
uint64_t tsync_global_us(const tsync_t *t, uint64_t now_local_us);  /* global_gettime() */
uint64_t tsync_local_for_global(const tsync_t *t, uint64_t global_us);
bool     tsync_is_synced(const tsync_t *t);
void     tsync_status(const tsync_t *t, int32_t *drift_ppb, int32_t *last_corr_us);

#endif /* TSYNC_H */
