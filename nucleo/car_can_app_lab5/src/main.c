/*
 * car_can_app - Lab 3/4/5 zone firmware for N2 (steering) and N3 (drivetrain)
 * Lab 5: Raft (raft_node.c, src/raft/), time sync, PPS (pps.c)
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
#include "raft_node.h"
#include <stm32_ll_rcc.h>

int main(void)
{
    printk("car_can_app N%d (%s), Lab 5\n", NODE_ID, NODE_ID == 2 ? "front / steering" : "rear / drivetrain");
    /* Why did we start? (RCC_CSR; the flags survive a reset until cleared) */
    printk("reset cause:%s%s%s%s%s%s\n",
           LL_RCC_IsActiveFlag_IWDGRST() ? " WATCHDOG" : "", LL_RCC_IsActiveFlag_SFTRST() ? " software" : "",
           LL_RCC_IsActiveFlag_PORRST() ? " power-on" : "", LL_RCC_IsActiveFlag_BORRST() ? " brown-out" : "",
           LL_RCC_IsActiveFlag_PINRST() ? " reset-pin" : "", LL_RCC_IsActiveFlag_WWDGRST() ? " window-wdg" : "");
    LL_RCC_ClearResetFlags();

    state_init();
    if (raft_node_init() != 0) {                /* before CAN: its queue takes the first frames */
        printk("raft init failed - check the LEADER / SYNC / REF_RX pins\n");
        return 0;
    }
    if (node_init() != 0 || blink_init() != 0 || relay_init() != 0 || pps_init() != 0) {
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
