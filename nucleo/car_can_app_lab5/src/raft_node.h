/*
 * raft_node.h - the Raft thread on a Nucleo (Lab 5)
 *
 * One thread owns raft_core and tsync. Everything else talks to it through
 * a queue (frames, TX-complete stamps, commands) or reads its published
 * state under a spinlock. Buses here are 1 = A, 2 = B (catalog convention).
 */
#ifndef RAFT_NODE_H
#define RAFT_NODE_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/can.h>

#include "raft/raft_core.h"

int raft_node_init(void);

/* Local monotonic clock, microseconds (64-bit cycle counter) */
uint64_t raft_node_local_us(void);

/* From the CAN layer (any thread). bus = 1 (A) or 2 (B).
 * first = false: a duplicate copy (other bus or relayed); only REF copies are
 * posted that way, for time sync. */
void raft_node_post_frame(uint8_t bus, const struct can_frame *f, uint64_t rx_us, bool first);
void raft_node_post_ref_txdone(uint8_t bus, uint64_t tx_us);

/* Commands (any thread) */
void raft_node_note_test_fault(uint8_t zone);       /* FAULT reason test button */
bool raft_node_report_event(uint8_t code);          /* APP_EVENT from this node */

/* Time (any thread) */
uint64_t raft_node_global_us(uint64_t local_us);    /* global_gettime() at a local time */
uint64_t raft_node_global_now(void);
uint64_t raft_node_local_for_global(uint64_t global_us);

/* Relay decision for a frame from `sender` that arrived on `from_bus`.
 * Returns the bus to forward on (1 or 2), or 0 for none. */
uint8_t raft_node_relay_bus(uint8_t sender, uint8_t from_bus, bool already_relayed);

struct raft_node_status {
    uint8_t term, leader, role;
    bool is_leader, silent, synced, ref_fresh;
    uint16_t last, commit;
    raft_net_t net;
    bool failure;
    int32_t drift_ppb, last_corr_us;
    uint32_t steps, slews;
    uint32_t ref_age_ms;
    uint32_t frames_in, frames_out, out_dropped;
    uint32_t no_stamp, no_match, reused, stale, big, outliers;
    int32_t bias_a_us, bias_b_us;       /* per-bus mean raw correction (tsync) */
    uint8_t last_cycle, last_sbus;      /* bytes 2 and 3 (low bits) of the last REF from the leader */
};
void raft_node_get_status(struct raft_node_status *s);
bool raft_node_failure(void);

#endif /* RAFT_NODE_H */
