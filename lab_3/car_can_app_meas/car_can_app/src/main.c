/*
 * car_can_app - Lab 3 zone firmware for N2 (steering) and N3 (drivetrain)
 *
 * Build one image per zone:
 *   west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=2
 *   west build -p always -b nucleo_f401re car_can_app -- -DNODE_ID=3
 *
 * Init order: outputs to their safe values first (power-up is the error
 * state), then CAN, then the periodic tasks.
 */

#include <zephyr/kernel.h>

#include "canbus.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "wcet.h"

int main(void)
{
    printk("car_can_app N%d (%s)\n", NODE_ID, NODE_ID == 2 ? "front / steering" : "rear / drivetrain");

    state_init();
    if (node_init() != 0 || blink_init() != 0 || relay_init() != 0) {
        printk("output init failed\n");
        return 0;
    }
    if (canbus_init() != 0) {
        printk("CAN init failed - check wiring, then reset\n");
        return 0;
    }
    wcet_init();                                /* no-op unless built with -DWCET=1 */
    if (button_init() != 0 || monitor_init() != 0) {
        printk("monitor / button init failed\n");
        return 0;
    }
    printk("running\n");
    return 0;
}
