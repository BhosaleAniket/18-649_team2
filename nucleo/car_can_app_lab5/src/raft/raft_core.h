/*
 * raft_core.h - pure Raft core (catalog v0.8). No OS calls, no allocation.
 *
 * The caller owns time (local microseconds), storage and the CAN driver:
 *   - feed every received Raft frame to raft_on_frame()
 *   - call raft_on_tick() at least every 1 ms
 *   - send every frame left in raft_out_t on the buses in its bus_mask
 * Both functions only append to `out`; the caller clears it (out->n = 0).
 *
 * Storage: `raft_t` holds the state; the log lives in caller memory
 * passed to raft_init() (RAFT_LOG_BYTES(log_size) bytes).
 */
#ifndef RAFT_CORE_H
#define RAFT_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "raft_proto.h"

typedef struct {
    uint8_t  n_nodes;               /* 3 */
    uint8_t  self_id;               /* 1..3 */
    uint8_t  default_leader;        /* 0 = none (LEADER=random) */
    uint16_t heartbeat_ms;          /* 20: REF period */
    uint16_t append_period_ms;      /* 100: empty APPEND period */
    uint16_t boot_listen_ms;        /* 100 */
    uint8_t  missed_heartbeats;     /* 3 */
    uint16_t election_jitter_ms;    /* 39 */
    uint16_t catchup_pace_ms;       /* 2 */
    uint16_t report_retry_ms;       /* 50 */
    uint32_t election_seed;
    uint16_t log_size;              /* entries */
    bool     debug;
} raft_config_t;

typedef struct { uint16_t id; uint8_t data[8]; uint8_t bus_mask; } raft_frame_t;  /* bus_mask bit (bus-1) */
#define RAFT_OUT_MAX 12
typedef struct { raft_frame_t f[RAFT_OUT_MAX]; uint8_t n; } raft_out_t;

/* Network graph: alive bit (n-1); link[n-1] bit (bus-1) */
typedef struct { uint8_t alive; uint8_t link[3]; } raft_net_t;

/* One applied (committed) entry, in index order, exactly once per index */
typedef void (*raft_apply_cb)(void *ctx, uint16_t index, uint8_t term, uint8_t entry, bool replayed);
/* Debug line (no newline); only called when cfg.debug is set. May be NULL. */
typedef void (*raft_debug_cb)(void *ctx, const char *line);

/* Bytes of caller memory for the log: 2 per entry, plus 32 bytes of replayed marks for the
 * last 256 indexes (an entry is applied long before 256 newer ones arrive) */
#define RAFT_REPLAY_RING 256u
#define RAFT_LOG_BYTES(n) ((size_t)(n) * 2u + RAFT_REPLAY_RING / 8u)

enum raft_role { RAFT_FOLLOWER = 0, RAFT_CANDIDATE = 1, RAFT_LEADER = 2 };

#define RAFT_MAX_REPORTS 6

/* State. Fields are private to raft_core.c; read them through the functions below. */
typedef struct raft {
    raft_config_t cfg;
    uint8_t  *log_term;             /* [log_size] */
    uint8_t  *log_ent;              /* [log_size] */
    uint8_t  *log_replayed;         /* bitset, indexed by index % RAFT_REPLAY_RING */
    raft_apply_cb apply_cb;
    raft_debug_cb debug_cb;
    void     *ctx;

    uint64_t now;
    uint8_t  role;
    uint8_t  term;
    uint8_t  voted_for;
    uint8_t  leader;
    uint16_t last;                  /* last log index (0 = empty) */
    uint16_t commit;
    uint16_t applied;
    uint32_t rng;

    /* boot / rejoin */
    bool     listening;             /* listen before acting */
    bool     claim_after_listen;    /* default leader at boot */
    uint64_t listen_until;
    bool     wait_first_ref;        /* default-leader mode: no campaigning before any leader is heard */
    bool     silent;

    /* timers */
    uint64_t election_deadline;
    uint64_t last_leader_contact;   /* 0 = never */

    /* elections */
    bool     prevote;               /* candidate in pre-vote phase */
    uint8_t  prevote_term;
    uint8_t  votes;                 /* node bitmask */

    /* leader */
    uint16_t next[4], match[4];     /* index by node id */
    uint64_t next_catchup[4];
    uint16_t bcast_next;            /* next new entry to broadcast */
    uint64_t next_bcast_time;
    uint64_t next_append_time;
    bool     commit_changed;

    /* client */
    uint16_t leader_last;           /* leader's last index, from its last empty APPEND */
    bool     leader_last_known;
    uint8_t  own_links;             /* from this node's own DEG detection */
    bool     own_links_known;
    uint16_t report_id;
    struct { uint8_t entry; uint16_t id; uint16_t min_index; uint64_t next_send; uint64_t give_up; bool used; } rep[RAFT_MAX_REPORTS];
    uint16_t last_report_id[4];     /* leader: last appended report id per reporter */
    bool     last_report_valid[4];

    /* leader-side observations */
    uint8_t  obs_alive;             /* nodes whose heartbeat is current */
    bool     obs_alive_known;
    bool     obs_cockpit_ok;
    bool     obs_cockpit_known;
    uint64_t test_pending_until[4]; /* test-button press waiting to be logged, by zone */

    /* applied state machine */
    raft_net_t net;
    bool     failure;               /* committed COCKPIT_ERROR */
} raft_t;

void raft_init(raft_t *r, const raft_config_t *cfg, void *log_mem, uint64_t now_us,
               raft_apply_cb cb, raft_debug_cb dbg, void *ctx);

/* Every Raft ID (REF included: term and election timer only; REF timing goes to tsync) */
void raft_on_frame(raft_t *r, uint8_t bus, uint16_t id, const uint8_t data[8],
                   uint64_t rx_time_us, raft_out_t *out);
void raft_on_tick(raft_t *r, uint64_t now_us, raft_out_t *out);

/* Inputs from the node's own detection */
void raft_set_own_links(raft_t *r, uint8_t link_mask);          /* this node's DEG detection; isolation rule + LINK reports */
void raft_set_silent(raft_t *r, bool silent);                   /* test button: send nothing, no election timer */
void raft_observe_peers(raft_t *r, uint8_t alive_mask);         /* heartbeats current (self bit ignored); leader writes NODE entries */
void raft_observe_cockpit(raft_t *r, bool ok);                  /* HB_N1 state (or the Pi's own); leader writes COCKPIT entries */
void raft_note_test_fault(raft_t *r, uint8_t zone);             /* FAULT reason test button seen; leader writes APP_EVENT */

/* Client: report an entry (APP_EVENT etc.). False if the queue is full. */
bool raft_report(raft_t *r, uint8_t entry);

/* Queries */
raft_net_t raft_get_network(const raft_t *r);
bool    raft_failure_mode(const raft_t *r);
bool    raft_is_leader(const raft_t *r);
uint8_t raft_leader(const raft_t *r);       /* 0 = unknown */
uint8_t raft_term(const raft_t *r);
uint8_t raft_role(const raft_t *r);
uint16_t raft_last_index(const raft_t *r);
uint16_t raft_commit_index(const raft_t *r);
bool    raft_log_entry(const raft_t *r, uint16_t index, uint8_t *term, uint8_t *entry);

#endif /* RAFT_CORE_H */
