/*
 * link.c - Pi <-> STM32 serial link
 *
 * RX: interrupt-driven byte parser (ISR context). Valid commands are stored
 *     under a spinlock, the CMD_RX test point toggles, and the control thread
 *     is woken through a semaphore so it can act within the 2 ms budget.
 * TX: status heartbeat thread, one frame every 20 ms, synced to a k_timer so
 *     the period does not drift.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>

#include "link.h"
#include "link_proto.h"
#include "control.h"
#include "config.h"

static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_comms));
static const struct gpio_dt_spec tp_cmd_rx = GPIO_DT_SPEC_GET(DT_NODELABEL(tp_cmd_rx), gpios);

K_SEM_DEFINE(link_cmd_sem, 0, 1);

/* ------------------------------------------------------------------ */
/* Shared state: written by the RX ISR, read by threads                */
/* ------------------------------------------------------------------ */

/* A spinlock is Zephyr's primitive for data shared between an ISR and
 * threads: it masks interrupts while held, so a reader can never see a
 * half-updated struct. volatile would not give that guarantee. */
static struct k_spinlock cmd_lock;
static struct link_cmd latest;

static atomic_t bad_frames = ATOMIC_INIT(0);

/* ------------------------------------------------------------------ */
/* RX parser - ISR context. No printing, no blocking.                 */
/* ------------------------------------------------------------------ */

enum rx_state { WAIT_SYNC0, WAIT_SYNC1, READ_BODY };

static enum rx_state rx_state = WAIT_SYNC0;
static uint8_t body[CMD_LEN - 2];       /* type .. crc, 8 bytes */
static uint8_t body_idx;
static int64_t last_byte_ms;

static void handle_frame(const uint8_t *b)
{
    /* b[0]=type b[1]=seq b[2..3]=steer b[4]=thr b[5]=brk b[6]=flags b[7]=crc */
    if (b[0] != CMD_TYPE ||
        link_crc8(b, 7) != b[7] ||
        (b[6] & CMD_FLAG_RESERVED_MASK) != 0) {
        atomic_inc(&bad_frames);
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&cmd_lock);
    latest.valid = true;
    latest.seq   = b[1];
    latest.steer = (int16_t)(b[2] | (b[3] << 8));
    latest.thr   = b[4];
    latest.brk   = b[5];
    latest.flags = b[6];
    latest.rx_ms = k_uptime_get();
    k_spin_unlock(&cmd_lock, key);

    gpio_pin_toggle_dt(&tp_cmd_rx);     /* CMD_RX test point */
    k_sem_give(&link_cmd_sem);          /* wake the control thread now */
}

static void feed_byte(uint8_t byte, int64_t now)
{
    /* A long gap in the middle of a frame means it was truncated. */
    if (rx_state == READ_BODY && (now - last_byte_ms) > BYTE_TIMEOUT_MS) {
        atomic_inc(&bad_frames);
        rx_state = WAIT_SYNC0;
    }
    last_byte_ms = now;

    switch (rx_state) {
    case WAIT_SYNC0:
        if (byte == LINK_SYNC0) {
            rx_state = WAIT_SYNC1;
        }
        break;

    case WAIT_SYNC1:
        if (byte == LINK_SYNC1) {
            rx_state = READ_BODY;
            body_idx = 0;
        } else if (byte != LINK_SYNC0) {
            rx_state = WAIT_SYNC0;      /* 0xAA again: stay, it may be the real start */
        }
        break;

    case READ_BODY:
        body[body_idx++] = byte;
        if (body_idx >= sizeof(body)) {
            handle_frame(body);
            rx_state = WAIT_SYNC0;
        }
        break;
    }
}

static void uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    uart_irq_update(dev);
    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t byte;
    while (uart_fifo_read(dev, &byte, 1) == 1) {
        feed_byte(byte, k_uptime_get());
    }
}

/* ------------------------------------------------------------------ */
/* Accessors - thread context                                         */
/* ------------------------------------------------------------------ */

void link_get_command(struct link_cmd *out)
{
    k_spinlock_key_t key = k_spin_lock(&cmd_lock);
    *out = latest;
    k_spin_unlock(&cmd_lock, key);
}

uint32_t link_bad_frames(void)
{
    return (uint32_t)atomic_get(&bad_frames);
}

/* ------------------------------------------------------------------ */
/* Status heartbeat thread                                            */
/* ------------------------------------------------------------------ */

K_TIMER_DEFINE(status_timer, NULL, NULL);
K_THREAD_STACK_DEFINE(status_stack, STACK_STATUS);
static struct k_thread status_thread_data;

static void status_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    uint8_t seq = 0;
    k_timer_start(&status_timer, K_MSEC(STATUS_PERIOD_MS), K_MSEC(STATUS_PERIOD_MS));

    while (1) {
        /* Blocks until the next expiry. Expiries are scheduled from the
         * previous one, so the 20 ms period does not drift. */
        k_timer_status_sync(&status_timer);

        uint8_t f[STATUS_LEN];
        uint16_t i_left = 0, i_right = 0, i_servo = 0;   /* TODO: current sensors */

        f[0]  = LINK_SYNC0;
        f[1]  = LINK_SYNC1;
        f[2]  = STATUS_TYPE;
        f[3]  = seq++;
        f[4]  = control_get_state();
        f[5]  = control_get_faults();
        f[6]  = (uint8_t)(i_left & 0xFF);
        f[7]  = (uint8_t)(i_left >> 8);
        f[8]  = (uint8_t)(i_right & 0xFF);
        f[9]  = (uint8_t)(i_right >> 8);
        f[10] = (uint8_t)(i_servo & 0xFF);
        f[11] = (uint8_t)(i_servo >> 8);
        f[12] = link_crc8(&f[2], 10);

        /* Polled TX: ~1.1 ms for 13 bytes. Runs in this thread, so the
         * higher-priority control thread can still preempt it. */
        for (int i = 0; i < STATUS_LEN; i++) {
            uart_poll_out(uart_dev, f[i]);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

int link_init(void)
{
    if (!device_is_ready(uart_dev) || !gpio_is_ready_dt(&tp_cmd_rx)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&tp_cmd_rx, GPIO_OUTPUT_INACTIVE);

    uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
    uart_irq_rx_enable(uart_dev);

    k_thread_create(&status_thread_data, status_stack,
                    K_THREAD_STACK_SIZEOF(status_stack),
                    status_thread, NULL, NULL, NULL,
                    PRIO_STATUS, 0, K_NO_WAIT);
    k_thread_name_set(&status_thread_data, "status");
    return 0;
}