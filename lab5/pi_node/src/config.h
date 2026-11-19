/*
 * config.h - reads raft_config.json (the one shared config file) for the Pi.
 */
#ifndef PI_CONFIG_H
#define PI_CONFIG_H

#include <stddef.h>
#include "raft_core.h"

typedef struct {               /* Pi-only settings, under "pi" in the JSON */
    int32_t  ref_stamp_adjust_us;  /* subtracted from the RECV_OWN_MSGS stamp (scope-measured) */
    int32_t  rx_skew_b_us;         /* added to bus-B receive times for time sync (B vs A delay) */
    bool     tsync_trace;          /* print every time-sync correction (report data) */
    int      pps_gpio;             /* BCM number of the PPS pin */
    int      rt_priority;          /* SCHED_FIFO priority of the C threads, 0 = none */
    int      cpu;                  /* pin the C threads to this CPU, -1 = no pinning */
    uint32_t rx_latency_us;
    uint32_t relay_delay_us;
} pi_extra_t;

/* Fills rc and px. Environment overrides: LEADER=cockpit|steering|drivetrain|random|none,
 * ELECTION_SEED=<n>. Returns 0, or -1 with a message in err. */
int config_load(const char *path, raft_config_t *rc, pi_extra_t *px, char *err, size_t errlen);

#endif
