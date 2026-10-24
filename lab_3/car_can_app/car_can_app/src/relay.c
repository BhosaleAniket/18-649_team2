/*
 * relay.c - RELAY, on both STM32 zones (N2 and N3)
 *
 * Design (Part 4): every first copy that DISPATCH accepts from another zone
 * is forwarded, unchanged, on the bus it did not arrive on.
 *   Who relays, when: both STM32s, always. No fault detection, so the
 *     relayed path already exists when a jumper is pulled (no 55-75 ms gap
 *     waiting for DEG detection). Whichever STM32 is still on both buses
 *     carries the traffic: N2 in checkoff cases 1-2, N3 in cases 3-4.
 *   Loops: only the FIRST accepted copy of an (ID, sender, seq) is relayed.
 *     A relayed copy is bit-for-bit the original (same ID, seq, CRC), so
 *     every receiver - including the other relay - drops it as a duplicate
 *     and never relays it again. A node never relays its own frames (it
 *     drops anything whose sender is itself). Two relays sending the same
 *     frame at the same instant send identical bits, so arbitration merges
 *     them.
 *   Priority: N2 RELAY = 0, the top preemptible priority, because in the
 *     analyzed case (N1 off B, N3 off A) it is a stage of N3's 2 ms brake
 *     chain. N3 RELAY = 4, below DRIVE and BLINK_R: N3 only ever relays
 *     N1<->N2 traffic (steering, blink, heartbeats, FORCE), whose deadlines
 *     are 20 ms or more, and it must not delay N3's own brake path.
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
    uint8_t from_bus;
};

K_MSGQ_DEFINE(relay_q, sizeof(struct relay_item), 16, 4);
static atomic_t relay_count, relay_refused, relay_overflow;

void relay_post(const struct can_frame *frame, int from_bus)
{
    struct relay_item it = { .frame = *frame, .from_bus = (uint8_t)from_bus };
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
        int out = (it.from_bus == BUS_A) ? BUS_B : BUS_A;

        if (canbus_send_one(out, &it.frame) == 0) {
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
