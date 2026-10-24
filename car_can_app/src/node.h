/*
 * node.h - what each zone (node2.c / node3.c) provides to the common code
 */
#ifndef NODE_H
#define NODE_H

#include <stdint.h>

int node_init(void);

/* A new valid CTRL is available (state_get_ctrl): wake STEER / DRIVE. */
void node_on_ctrl(void);

/* Drive the zone's outputs to their fail-safe values now
 * (N2: servo centered; N3: motor braked). Called on entering error,
 * from thread context. */
void node_apply_fail_safe(void);

/* Watchdog: bit set by each periodic thread that must keep running */
#define WDT_BIT_HB_TX   (1u << 0)
#define WDT_BIT_HB_MON  (1u << 1)
#define WDT_BIT_NODE    (1u << 2)   /* PID on N3, FORCE_TX on N2 */
#define WDT_ALL_BITS    (WDT_BIT_HB_TX | WDT_BIT_HB_MON | WDT_BIT_NODE)
void wdt_checkin(uint32_t bit);

/* Blinker outputs (blink.c) */
int blink_init(void);
void blink_on_frame(uint8_t state, uint8_t phase);
void blink_kick(void);      /* re-evaluate now (error state changed) */
void blink_sync_info(uint8_t out[3]);   /* N2: hazard timing for its heartbeat */
void blink_follow(const uint8_t in[3]); /* N3: align to N2's hazard timing */

int button_init(void);
int monitor_init(void);

#endif /* NODE_H */
