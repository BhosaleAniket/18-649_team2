/*
 * config.h - priorities, periods and thresholds, in one place
 *
 * Zephyr: lower number = higher priority. Negative = cooperative.
 * Priorities are deadline-monotonic over the Lab 3 template task set.
 *
 *  Task           Node   Kind                    Period/min IA  Deadline        Prio
 *  -------------  -----  ----------------------  -------------  --------------  ----
 *  MCP2515 INT    both   driver thread (coop)    per frame      -               coop 2
 *    + rx_cb      both   our callback inside it  per frame      (CAN_RX 0.3 ms) (same)
 *  RELAY          N2     on accepted frame       per frame      brake path      0
 *  DISPATCH       both   on rx queue             per frame      0.3 ms          1
 *  DRIVE          N3     on CTRL                 20 ms          0.5 ms local    2
 *  BLINK_F        N2     on BLINK (+20 ms tick)  20 ms          0.5 ms          2
 *  BLINK_R        N3     on BLINK (+20 ms tick)  20 ms          0.5 ms          3
 *  STEER          N2     on CTRL                 20 ms          1 ms local      3
 *  RELAY          N3     on accepted frame       per frame      steering path   4
 *  TEST_BTN       both   GPIO ISR + thread       50 ms          10 ms           5
 *  PID            N3     timer                   10 ms          10 ms           6
 *  HB_MON         both   timer                   20 ms          20 ms           7
 *  HB_TX          both   timer                   20 ms          20 ms           8
 *  FORCE_TX       N2     timer                   50 ms          50 ms           9
 *  REPORT         both   timer (printk)          1000 ms        best effort     12
 *  WATCHDOG       both   timer                   100 ms         IWDG 500 ms     13
 *
 * Lab 5 additions:
 *  BLINK, PPS     both   global-time edges       125-1000 ms    edge +-1 ms     coop (-1)
 *  RAFT           both   queue + 1 ms tick       1 ms           REF 20 ms       0
 *  RAFT_LOG       both   queue (printk)          -              best effort     11
 * BLINK and PPS sleep to just before their edge and spin the last
 * EDGE_WAKE_EARLY_US as the lowest cooperative priority, so no thread can
 * preempt them in that window (the CAN driver thread waits at most that long).
 */
#ifndef CONFIG_H
#define CONFIG_H

#ifndef NODE_ID
#error "NODE_ID must be defined (2 or 3) - set by CMakeLists.txt"
#endif

/* ---- Priorities ---- */
#if NODE_ID == 2
#define PRIO_RELAY      0       /* N2 relays on N3's brake path (checkoff cases 1-2): top */
#define PRIO_DISPATCH   1
#define PRIO_BLINK      2
#define PRIO_ACTUATE    3       /* STEER */
#else
#define PRIO_DISPATCH   1
#define PRIO_ACTUATE    2       /* DRIVE */
#define PRIO_BLINK      3
#define PRIO_RELAY      4       /* N3 relays only N1<->N2 traffic (steering, blink, HB): below its own brake path */
#endif
#define PRIO_RAFT       0       /* REF every 20 ms; a late REF starts an election */
#define PRIO_EDGE       (-1)    /* lowest cooperative: blink and PPS edges */
#define PRIO_RAFTLOG    11
#define PRIO_BUTTON     5
#define PRIO_PID        6
#define PRIO_HB_MON     7
#define PRIO_HB_TX      8
#define PRIO_FORCE_TX   9
#define PRIO_REPORT     12
#define PRIO_WATCHDOG   13

/* ---- Stacks ---- */
#define STACK_SMALL     1024
#define STACK_MEDIUM    1536
#define STACK_REPORT    2048
#define STACK_RAFT      2048

/* ---- Periods ---- */
#define HB_PERIOD_MS        20      /* R4.5: 20 ms +/-10% */
#define HB_MON_PERIOD_MS    20
#define PID_PERIOD_MS       10
#define FORCE_PERIOD_MS     50
#define BLINK_TICK_MS       20      /* local fallback tick for hazards */
#define REPORT_PERIOD_MS    1000
#define WDT_FEED_PERIOD_MS  100
#define WDT_TIMEOUT_MS      500

/* ---- Fault detection (R4.5, R4.7) ----
 * DEG: no frame of any kind on that bus for DEG_TIMEOUT_MS. Every other
 *   node sends a heartbeat every 20 ms, so a healthy bus never goes quiet
 *   for more than ~22 ms. Worst-case detection = 35 + one HB_MON period
 *   (20) = 55 ms, inside the 60 ms checkoff limit.
 * ZONE: no heartbeat from that zone on either bus for ZONE_TIMEOUT_MS
 *   (three missed periods + margin for the +10% jitter).
 * CTRL: no valid CTRL for CTRL_TIMEOUT_MS (proxy stopped or cockpit gone). */
#define DEG_TIMEOUT_MS      35
#define ZONE_TIMEOUT_MS     66      /* 3 x 20 ms + 10 % (clarification: 66 ms) */
#define CTRL_TIMEOUT_MS     65

/* ---- Duplicate / stale filtering (per ID and sender) ----
 * Same 4-bit seq within DEDUP_WINDOW_MS = another copy of a frame already
 *   accepted (the other bus, or a relay) -> drop. The window must cover the
 *   slowest copy (a relayed copy that waited behind other frames), and stay
 *   below the time the counter takes to come round again: 16 frames x 18 ms
 *   minimum interarrival = 288 ms. 150 ms does both. (10 ms was too short:
 *   a late copy was taken as a new frame - a late BLINK copy set the lamps
 *   back to an old phase.)
 * seq 1..7 behind the last accepted one within STALE_WINDOW_MS = an old
 *   frame flushed out of a TX buffer after a jumper went back in -> drop.
 *   After STALE_WINDOW_MS of silence anything is accepted (sender reboot). */
#define DEDUP_WINDOW_MS     150
#define STALE_WINDOW_MS     250

/* ---- Error state ---- */
#define BADCMD_HOLD_MS      1000    /* stay in error this long after the last bad CTRL */
#define REMOTE_HOLD_MS      100     /* a FAULT from another zone holds us in error at least this long */

/* ---- Test button ---- */
#define BUTTON_DEBOUNCE_MS  50
#define DOUBLE_PRESS_MS     500     /* clarification: double press within 500 ms restores */

/* ---- Lab 5: blinkers and edges on global time ----
 * Rates from the checkoff: hazards at 4 Hz with the cockpit lost (step 6),
 * back to 2 Hz when it returns (step 7). Open question to the TA: confirm
 * which rate goes with which state. */
#define BLINK_HALF_US_NORMAL    250000U     /* 2 Hz */
#define BLINK_HALF_US_FAILURE   125000U     /* 4 Hz */
#define EDGE_WAKE_EARLY_US      200U        /* sleep until this long before an edge (2 kernel ticks), then spin */

/* ---- Steering (N2) ---- */
#define STEER_FULL_SCALE    8192    /* wheel units for full chassis lock (Lab 2 value) */
#define SERVO_LEFT_DEG      30.0f
#define SERVO_CENTER_DEG    90.0f
#define SERVO_RIGHT_DEG     150.0f

/* ---- Drive (N3) ---- */
#define BRAKE_ON_THRESHOLD  25      /* brake u8 above this = brake (about 10% pedal) */
#define MAX_DUTY            0.95f
#define MOTOR_PERIOD_NS     50000U  /* 20 kHz */

#endif /* CONFIG_H */
