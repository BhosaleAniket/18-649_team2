/*
 * relay.c - RELAY, on both STM32 zones (N2 and N3)
 *
 * Lab 5: DISPATCH decides per frame with the shared route rule
 * (route_relay_mask via raft_node_relay_bus), the same code the Pi runs:
 * forward to the other bus only if this node is on both buses, the sender
 * cannot have put the frame there itself (committed graph, or the sender's
 * own bus-health bits in its heartbeat), and some other node needs it.
 * Raft frames arrive here with RELAYED already set (never relayed again);
 * Lab 3 frames are forwarded bit for bit and stopped from looping by the
 * Lab 3 (ID, sender, seq) dedup, as before.
 *
 * Priorities are unchanged from Lab 4: N2 RELAY = 0 (on N3's brake path in
 * the analysed case), N3 RELAY = 4 (below its own DRIVE and BLINK_R).
 *
 * RELAY_TX toggles AFTER the send on the other bus returns (clarification
 * probe point for measurement #6).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "canbus.h"
#include "config.h"
#include "wcet.h"

static const struct gpio_dt_spec tp_relay = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_relay_tx), gpios);

struct relay_item {
    struct can_frame frame;
    uint8_t out_bus;
};

K_MSGQ_DEFINE(relay_q, sizeof(struct relay_item), 24, 4);
static atomic_t relay_count, relay_refused, relay_overflow;

void relay_post_to(const struct can_frame *frame, int out_bus)
{
    struct relay_item it = { .frame = *frame, .out_bus = (uint8_t)out_bus };
    if (k_msgq_put(&relay_q, &it, K_NO_WAIT) != 0) {
        atomic_inc(&relay_overflow);
    }
}

uint32_t relay_forwarded(void)
{
    return (uint32_t)atomic_get(&relay_count);
}

K_THREAD_STACK_DEFINE(relay_stack, STACK_SMALL);
static struct k_thread relay_thread;

static void relay_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    struct relay_item it;

    while (1) {
        k_msgq_get(&relay_q, &it, K_FOREVER);
        WCET_BEGIN(relay);
        if (canbus_send_one(it.out_bus, &it.frame) == 0) {
            atomic_inc(&relay_count);
            gpio_pin_toggle_dt(&tp_relay);      /* RELAY_TX: the send on the other bus returned */
        } else {
            atomic_inc(&relay_refused);         /* that bus is degraded / buffers full */
        }
        WCET_END(W_RELAY, relay);
    }
}

int relay_init(void)
{
    if (!gpio_is_ready_dt(&tp_relay)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&tp_relay, GPIO_OUTPUT_INACTIVE);
    k_thread_create(&relay_thread, relay_stack, K_THREAD_STACK_SIZEOF(relay_stack),
                    relay_fn, NULL, NULL, NULL, PRIO_RELAY, 0, K_NO_WAIT);
    k_thread_name_set(&relay_thread, "relay");
    return 0;
}
