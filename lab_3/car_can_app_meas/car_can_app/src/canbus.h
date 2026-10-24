#ifndef CANBUS_H
#define CANBUS_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/can.h>

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

/* relay.c (both zones): hand an accepted frame to RELAY */
void relay_post(const struct can_frame *frame, int from_bus);
int relay_init(void);
uint32_t relay_forwarded(void);

#endif /* CANBUS_H */
