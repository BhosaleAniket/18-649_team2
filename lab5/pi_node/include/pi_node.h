/*
 * pi_node.h - Pi only (catalog v0.7). The only API cockpit.py calls (ctypes).
 * All functions are thread-safe. Return values: 0 = ok / got one, 1 = timeout, < 0 = error.
 */
#ifndef PI_NODE_H
#define PI_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "raft_core.h"

#ifdef __cplusplus
extern "C" {
#endif

int  pi_node_start(const char *config_json_path);   /* opens canA/canB, starts the SCHED_FIFO threads */
void pi_node_stop(void);
int  pi_node_send_app(uint16_t id, const uint8_t data[8]);         /* CTRL, BLINK: sent and relayed by C */
int  pi_node_poll_app(uint16_t *id, uint8_t data[8], uint8_t *bus, int timeout_ms);
void pi_node_set_cockpit_state(bool ok, uint8_t err_bits);         /* feeds HB_N1 and COCKPIT reports */
int  pi_node_report_event(uint8_t code);                            /* APP_EVENT from the cockpit */
void pi_node_status(raft_net_t *net, uint8_t *leader, uint8_t *term, bool *failure,
                    uint8_t *bus_ok_mask, int32_t *drift_ppb);
int  pi_node_poll_log(uint16_t *index, uint8_t *term, uint8_t *entry, bool *replayed, int timeout_ms);

/* Pi-only additions (not used by the Nucleos, no catalog change) */
uint64_t pi_node_global_us(void);   /* global_gettime(): global time in us now (0 if not started) */

#ifdef __cplusplus
}
#endif
#endif /* PI_NODE_H */
