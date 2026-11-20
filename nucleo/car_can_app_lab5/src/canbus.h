#ifndef CANBUS_H
#define CANBUS_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/can.h>

#include "raft/raft_core.h"

#define BUS_A 0
#define BUS_B 1
#define NUM_BUSES 2

/* Starts both controllers, registers the catalog filters and the
 * DISPATCH thread. */
int canbus_init(void);

/* Send helper: fills the sequence counter for this ID (bumped once),
 * then queues the frame on bus A and bus B back to back.
 * is_heartbeat: heartbeats still go out on a degraded bus (they are the
 * probe that lets the bus recover); everything else is held back there. */
void canbus_send_both(uint16_t id, uint8_t data[8], bool is_heartbeat);

/* Lab 5: send a Raft frame (sequence counter filled here) on the buses in
 * its bus_mask that are not degraded. is_ref: stamp TX-complete for tsync.
 * Returns 0 if it was queued on at least one bus. */
int canbus_send_raft(const raft_frame_t *f, bool is_ref);

/* Queue an unchanged frame on one bus (RELAY). Returns can_send's result,
 * or -ENETDOWN if that bus is degraded. */
int canbus_send_one(int bus, const struct can_frame *frame);

/* Uptime (ms, 32-bit) of the last frame received on this bus */
uint32_t canbus_last_rx_ms(int bus);

/* Called by HB_MON on a health change. Entering degraded flushes the
 * controller's pending TX buffers (stop + start) and holds back
 * non-heartbeat traffic on that bus until frames arrive again. */
void canbus_set_degraded(int bus, bool degraded);
bool canbus_is_degraded(int bus);

/* Called by HB_MON every tick while a bus is degraded: if the controller is
 * stopped (a failed can_start), start it again, with backoff. Bus-off is left
 * to the controller's own recovery (CAN fault confinement). */
void canbus_recover(int bus);

/* Controller state for the 1 s report */
struct canbus_ctrl_diag {
    bool state_ok;
    uint8_t state;          /* enum can_state */
    uint8_t tec, rec;
    uint32_t restarts, restart_fail;
    bool int_asserted;      /* MCP2515 INT line asserted right now */
    uint32_t int_kicks;     /* stalled interrupts re-triggered */
    uint32_t tx_stuck;      /* restarts because no TX completed while sends were refused */
};

/* Called by HB_MON every tick for both buses: restart a controller whose TX
 * slots are stuck (receiving fine, nothing completes, every send refused). */
void canbus_check_tx(int bus);

/* Level of the MCP2515 INT pin (1 = asserted), safe from an ISR */
int canbus_int_pin(int bus);

/* Called by HB_MON every tick for both buses: re-trigger a stalled MCP2515
 * interrupt (INT asserted on two ticks in a row). */
void canbus_check_int(int bus);
void canbus_get_ctrl_diag(int bus, struct canbus_ctrl_diag *d);

struct canbus_stats {
    uint32_t rx[NUM_BUSES];
    uint32_t tx_ok[NUM_BUSES];
    uint32_t tx_err[NUM_BUSES];
    uint32_t tx_refused[NUM_BUSES];
    uint32_t dup_dropped;
    uint32_t stale_dropped;
    uint32_t rxq_overflow;
};
void canbus_get_stats(struct canbus_stats *out);
const struct device *canbus_dev(int bus);

/* Implemented by the application (state.c): one accepted, de-duplicated
 * frame from another zone. Runs in the DISPATCH thread. */
void app_on_frame(const struct can_frame *frame, int bus, uint8_t src);

/* relay.c (both zones): hand a frame to RELAY for bus `out_bus` (0 = A, 1 = B) */
void relay_post_to(const struct can_frame *frame, int out_bus);
int relay_init(void);
uint32_t relay_forwarded(void);

#endif /* CANBUS_H */
