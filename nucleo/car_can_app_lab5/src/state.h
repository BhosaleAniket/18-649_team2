#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

/* Latest valid CTRL */
struct ctrl_cmd {
    bool valid;         /* at least one valid CTRL since boot */
    int16_t steer;
    uint8_t thr;
    uint8_t brk;
    uint8_t buttons;
    uint32_t t_ms;      /* uptime when accepted */
};

void state_init(void);

/* Error state: in error whenever any reason bit (ERR_*) is set.
 * Entering error applies the zone's fail-safe outputs immediately. */
void state_set(uint32_t reasons);
void state_clear(uint32_t reasons);
uint32_t state_reasons(void);
bool state_in_error(void);

/* Consistent snapshot of the latest valid CTRL */
void state_get_ctrl(struct ctrl_cmd *out);

/* What HB_MON needs about the other zones */
struct peer_info {
    uint32_t last_hb_ms;    /* 0 = never heard */
    uint8_t reasons;        /* ERR_* bits from its last heartbeat */
    uint8_t zone_state;     /* 0 error, 1 normal */
    uint8_t bus_health;     /* bit0 A ok, bit1 B ok, as it reports */
};
void state_get_peer(uint8_t zone, struct peer_info *out);
uint32_t state_last_remote_fault_ms(void);
uint32_t state_last_badcmd_ms(void);

/* Send FAULT (failing zone, reason) on both buses */
void state_send_fault(uint8_t failing_zone, uint8_t reason);

/* Diagnostics for the 1 s report: NORMAL->ERROR transitions, and how often
 * each reason bit (CTRL_LOST, ZONE_LOST, TEST, BADCMD, REMOTE) newly set */
void state_get_diag(uint32_t *entries, uint32_t counts[5]);

#endif /* STATE_H */
