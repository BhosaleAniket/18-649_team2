/*
 * config.h - task priorities, periods, and deadlines in one place.
 *
 * Zephyr: lower number = higher priority. All threads are preemptible,
 * so a higher-priority thread that becomes ready runs immediately.
 *
 *  Task             Kind        Period / trigger          Prio  Deadline
 *  ---------------  ----------  ------------------------  ----  ------------------
 *  UART RX parser   ISR         each received byte        IRQ   one byte time
 *  control          thread      new command OR 20 ms      1     2 ms brake/throttle
 *                                                                50 ms steering
 *  status TX        thread      20 ms, timer-synced       3     20 ms +/- 10%
 *  blinker logic    thread      20 ms                     5     100 ms
 *  blink output     timer ISR   250 ms                    IRQ   pattern edge
 *  monitor          thread      500 ms                    7     best effort
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- Priorities ---- */
#define PRIO_CONTROL   1
#define PRIO_STATUS    3
#define PRIO_BLINKER   5
#define PRIO_MONITOR   7

/* ---- Stack sizes ---- */
#define STACK_CONTROL  2048
#define STACK_STATUS   1024
#define STACK_BLINKER  1024
#define STACK_MONITOR  2048

/* ---- Periods ---- */
#define CONTROL_PERIOD_MS   20
#define STATUS_PERIOD_MS    20
#define BLINKER_PERIOD_MS   20
#define BLINK_TICK_MS       250     /* half-period of the 2 Hz hazard */
#define MONITOR_PERIOD_MS   500

/* ---- Link / fault handling ---- */
#define FAIL_SAFE_MS        60      /* 3 missed 20 ms frames */
#define BYTE_TIMEOUT_MS     5       /* gap inside a frame = truncated */
#define BADCMD_HOLD_MS      500     /* error lasts this long after a bad frame */

/* ---- Buttons (levels arrive in each command frame) ---- */
#define BUTTON_DEBOUNCE_MS  50      /* ignore level changes closer than this */
#define DOUBLE_PRESS_MS     800     /* two presses within this = double press */

/* ---- Steering ----
 * Raw wheel value (steer units) at which the chassis reaches full lock.
 * The raw range is +/-32767 over the wheel's whole rotation (~900 deg
 * lock to lock on the G923 at its default setting), so full scale at
 * 8192 means about +/-112 deg of wheel rotation reaches full chassis lock.
 * Beyond that the servo holds at its limit. Adjust to taste: smaller
 * = quicker steering. Blinker thresholds scale with this. */
#define STEER_FULL_SCALE    8192

#endif /* CONFIG_H */