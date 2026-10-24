/*
 * wcet.h - execution-time measurement for the Part 3 analysis
 *
 * Build with -DWCET=1 to enable; without it every macro compiles to nothing.
 *
 * For each instrumented task body it records, over all jobs since boot:
 *   pure  = CPU cycles the thread itself spent in the body (Zephyr thread
 *           runtime stats), i.e. C_i with preemption by other threads removed
 *   wall  = elapsed cycles from start to end, preemption included
 * The fixed cost of one measurement pair is calibrated at boot and
 * subtracted from "pure".
 *
 * RX_PATH is measured differently: an extra GPIO callback on each MCP2515
 * INT pin timestamps the falling edge, and rx_cb() records INT -> frame
 * delivered (driver ISR + driver thread wake-up + SPI read). That is the
 * part of CAN_RX inside the Zephyr driver; DISPATCH is the rest.
 *
 * The report thread prints one line per task every 5 s:
 *   WCET <name> n=<jobs> pure_max=<us> wall_max=<us>
 * parse_wcet.py turns a captured serial log into measured.json for rta.py.
 */
#ifndef WCET_H
#define WCET_H

#include <stdint.h>

enum wcet_id {
    W_RX_PATH,      /* INT edge -> rx_cb (wall only) */
    W_DISPATCH,     /* one frame through DISPATCH */
    W_DRIVE,
    W_STEER,
    W_BLINK,
    W_TEST_BTN,
    W_PID,
    W_HB_MON,
    W_HB_TX,        /* one heartbeat on both buses */
    W_FORCE_TX,     /* one FORCE on both buses */
    W_RELAY,        /* one forwarded frame */
    W_SPI_TX,       /* one can_send() call = TX buffer load over SPI */
    W_ACT_CS,       /* actuator-lock critical section */
    W_COUNT
};

#if defined(WCET) && WCET

#include <zephyr/kernel.h>

struct wcet_mark {
    uint64_t exec0;
    uint32_t t0;
};

void wcet_init(void);
void wcet_begin(struct wcet_mark *m);
void wcet_end(enum wcet_id id, const struct wcet_mark *m);
void wcet_record_wall(enum wcet_id id, uint32_t cycles);
void wcet_tx_delivered(int bus);        /* called from tx_cb: INT was a TX-done, not a frame */
void wcet_rx_delivered(int bus);        /* called from rx_cb */
void wcet_report(void);

#define WCET_BEGIN(name)    struct wcet_mark _wcet_##name; wcet_begin(&_wcet_##name)
#define WCET_END(id, name)  wcet_end((id), &_wcet_##name)

#else

static inline void wcet_init(void) {}
static inline void wcet_tx_delivered(int bus) { (void)bus; }
static inline void wcet_rx_delivered(int bus) { (void)bus; }
static inline void wcet_report(void) {}
#define WCET_BEGIN(name)    do { } while (0)
#define WCET_END(id, name)  do { } while (0)

#endif

#endif /* WCET_H */
