/*
 * pps.c - PPS output (Lab 5 Part 6.4)
 *
 * One rising edge on the PPS pin at every whole second of global_gettime().
 * Every node derives the edge from the same global time, so on a scope the
 * edges of all nodes line up; their spread is the sync error.
 *
 * The thread sleeps until EDGE_WAKE_EARLY_US before the edge and spins to
 * the exact microsecond. It is cooperative (PRIO_EDGE): nothing preempts it
 * in that window. Every wakeup recomputes the edge from the current model,
 * sleeps are capped at 1 s and the spin at 2 x EDGE_WAKE_EARLY_US, so a
 * clock step can never leave it spinning. The pin drops PPS_HIGH_MS later.
 * On each edge it queues a console line (printed by RAFT_LOG): global and
 * local time at the edge, the drift estimate and the correction applied at
 * the last REF.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "config.h"
#include "raft_node.h"

#define PPS_HIGH_MS 100

static const struct gpio_dt_spec pps_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_pps), gpios);

void raft_node_log_pps(uint64_t global_us, uint64_t local_us);

K_THREAD_STACK_DEFINE(pps_stack, STACK_SMALL);
static struct k_thread pps_thread;

static void pps_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    while (1) {
        /* Recompute the next whole second from the model as it is now (see
         * blink.c: an edge computed before a clock step is never reused) */
        uint64_t now = raft_node_local_us();
        uint64_t g = raft_node_global_us(now);
        uint64_t edge_g = (g / 1000000u + 1u) * 1000000u;
        uint64_t edge_l = raft_node_local_for_global(edge_g);

        if (edge_l <= now || edge_l - now > EDGE_WAKE_EARLY_US) {
            uint64_t wait = (edge_l > now + EDGE_WAKE_EARLY_US) ? edge_l - now - EDGE_WAKE_EARLY_US : 100u;
            if (wait > 1000000u) {
                wait = 1000000u;
            }
            k_sleep(K_USEC((uint32_t)wait));
            continue;
        }

        uint64_t spin_end = now + 2u * EDGE_WAKE_EARLY_US;
        uint64_t t;
        while ((t = raft_node_local_us()) < edge_l && t < spin_end) {
        }
        gpio_pin_set_dt(&pps_pin, 1);
        raft_node_log_pps(raft_node_global_us(t), t);
        k_msleep(PPS_HIGH_MS);
        gpio_pin_set_dt(&pps_pin, 0);
    }
}

int pps_init(void)
{
    if (!gpio_is_ready_dt(&pps_pin)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&pps_pin, GPIO_OUTPUT_INACTIVE);
    k_thread_create(&pps_thread, pps_stack, K_THREAD_STACK_SIZEOF(pps_stack),
                    pps_fn, NULL, NULL, NULL, PRIO_EDGE, 0, K_NO_WAIT);
    k_thread_name_set(&pps_thread, "pps");
    return 0;
}
