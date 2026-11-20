/*
 * button.c - TEST_BTN (R2.5): B1 / PC13
 *
 * ISR: on the press edge, accept it only if 50 ms have passed since the
 *      last accepted edge (leading-edge debounce, so there is no delay),
 *      then give a semaphore. No printing, no SPI.
 * Thread (prio 5):
 *      not in test error  -> enter it: ERR_TEST (fail-safe outputs are
 *                            applied inside state_set), then FAULT on both
 *                            buses. Local fail well inside 10 ms.
 *      already in test error and this press is within DOUBLE_PRESS_MS
 *      (500 ms) of the previous one -> double press: leave test error.
 *      While in test error HB_TX sends nothing (monitor.c), so the other
 *      zones see the FAULT at once and the missing heartbeats after that.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "catalog.h"
#include "config.h"
#include "node.h"
#include "state.h"
#include "wcet.h"

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback btn_cb;

K_SEM_DEFINE(btn_sem, 0, 4);
static uint32_t last_edge_ms;

static void btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
    uint32_t now = k_uptime_get_32();

    if ((now - last_edge_ms) < BUTTON_DEBOUNCE_MS) {
        return;
    }
    last_edge_ms = now;
    k_sem_give(&btn_sem);
}

K_THREAD_STACK_DEFINE(btn_stack, STACK_SMALL);
static struct k_thread btn_thread;

static void btn_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    uint32_t last_press_ms = 0;
    bool have_last = false;

    while (1) {
        k_sem_take(&btn_sem, K_FOREVER);
        uint32_t now = k_uptime_get_32();

        if ((state_reasons() & ERR_TEST) == 0) {
            WCET_BEGIN(btn);
            state_set(ERR_TEST);                        /* local fail */
            state_send_fault(NODE_ID, FAULT_TEST_BUTTON);
            WCET_END(W_TEST_BTN, btn);
            have_last = false;                          /* the failing press never starts a double press */
            continue;
        } else if (have_last && (now - last_press_ms) <= DOUBLE_PRESS_MS) {
            state_clear(ERR_TEST);                      /* double press: restore */
            have_last = false;
            continue;
        }
        last_press_ms = now;
        have_last = true;
    }
}

int button_init(void)
{
    if (!gpio_is_ready_dt(&btn)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb, btn_isr, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb);
    last_edge_ms = k_uptime_get_32() - BUTTON_DEBOUNCE_MS;

    k_thread_create(&btn_thread, btn_stack, K_THREAD_STACK_SIZEOF(btn_stack),
                    btn_fn, NULL, NULL, NULL, PRIO_BUTTON, 0, K_NO_WAIT);
    k_thread_name_set(&btn_thread, "test_btn");
    return 0;
}
