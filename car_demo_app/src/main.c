/*
 * car_app - Lab 2 integrated firmware (Nucleo-F401RE, Zephyr)
 *
 * Laptop proxy -> UDP -> Pi -> UART (USART1) -> this board -> motors,
 * servo, blinkers. Status heartbeat goes back to the Pi every 20 ms.
 *
 * Modules:
 *   link.c      RX parser ISR, status TX thread
 *   control.c   fault state, brake, PI speed control, servo (control thread)
 *   blinkers.c  turn signals, self-cancel, hazards (blinker thread + timer ISR)
 *   main.c      init, then the low-priority console monitor
 *
 * Priorities, periods and deadlines: see config.h.
 */

#include <zephyr/kernel.h>

#include "config.h"
#include "link.h"
#include "link_proto.h"
#include "control.h"
#include "blinkers.h"
#include "current.h"

static void print_faults(uint8_t f)
{
    if (f == 0) {
        printk("none");
        return;
    }
    if (f & ST_FAULT_POWERUP)  { printk("POWERUP "); }
    if (f & ST_FAULT_LINK)     { printk("LINK "); }
    if (f & ST_FAULT_BADCMD)   { printk("BADCMD "); }
    if (f & ST_FAULT_SELFTEST) { printk("SELFTEST "); }
}

int main(void)
{
    /* Actuators first, so the board is braked and hazards are on
     * before any command can arrive. */
    if (control_init() != 0 || blinkers_init() != 0 || current_init() != 0 ||
        link_init() != 0) {
        printk("ERROR: init failed\n");
        return 0;
    }

    k_thread_priority_set(k_current_get(), PRIO_MONITOR);
    printk("car_app running\n");

    /* Monitor: lowest priority, console only. All printing lives here so
     * it can never delay the time-critical threads. */
    uint8_t last_faults = 0xFF;
    uint8_t last_buttons = 0;
    int64_t next_print = 0;

    while (1) {
        uint8_t faults = control_get_faults();
        int64_t now = k_uptime_get();

        /* Button levels as received from the Pi, so presses are visible */
        struct link_cmd bc;
        link_get_command(&bc);
        uint8_t buttons = bc.flags & (CMD_FLAG_LEFT | CMD_FLAG_RIGHT | CMD_FLAG_TEST);
        if (buttons != last_buttons) {
            printk("[%u ms] buttons: left=%d right=%d test=%d\n", (uint32_t)now,
                   (buttons & CMD_FLAG_LEFT) != 0, (buttons & CMD_FLAG_RIGHT) != 0,
                   (buttons & CMD_FLAG_TEST) != 0);
            last_buttons = buttons;
        }

        if (faults != last_faults) {
            printk("[%u ms] state %s, faults: ", (uint32_t)now,
                   faults ? "ERROR" : "NORMAL");
            print_faults(faults);
            printk("\n");
            last_faults = faults;
        }

        if (now >= next_print) {
            next_print = now + MONITOR_PERIOD_MS;

            struct link_cmd cmd;
            struct control_telemetry t;
            struct current_readings cur;
            link_get_command(&cmd);
            control_get_telemetry(&t);
            current_get(&cur);

            printk("steer=%d thr=%u brk=%u flags=0x%02x | "
                   "target=%d rpm=%d duty=%d%%%s | I=%u/%u/%u mA%s | turn=%s | bad=%u\n",
                   cmd.steer, cmd.thr, cmd.brk, cmd.flags,
                   t.target_x10 / 10, t.rpm_x10 / 10, t.duty_pct,
                   t.braking ? " BRAKE" : "",
                   cur.left_ma, cur.right_ma, cur.servo_ma,
                   current_is_valid() ? "" : " (no zero)",
                   blinkers_turn_name(), link_bad_frames());
        }

        k_msleep(50);
    }

    return 0;
}