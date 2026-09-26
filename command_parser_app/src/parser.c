/*
Parser implementation for the STM32 side of the PI to STM32 link protocol
*/

#include "parser.h"
#include "link_proto.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#define BYTE_TIMEOUT_MS 5

static const struct device *uart_dev;

// state-machine definitions for the parser stages
typedef enum {
    WAIT_SYNC0, WAIT_SYNC1, WAIT_TYPE, READ_BODY
} parser_state_t;

static parser_state_t state = WAIT_SYNC0;

//TODO: Update this to handle the heartbeats as well
static uint8_t buf[CMD_LEN];   // CMD_LEN=10 is the larger of the two frames we care about here (RX side only handles CMD)
static uint8_t body_idx;
static uint8_t expected_len;
static int64_t last_byte_time_ms;

static volatile bool have_fresh_cmd = false;
static volatile bool last_frame_bad = false;
static cmd_frame_t latest_cmd;
static volatile int64_t last_valid_cmd_time_ms = 0;
static volatile bool ever_valid = false;

static void reset_parser(void)
{
    state = WAIT_SYNC0;
    body_idx = 0;
}

static void handle_complete_cmd_frame(void)
{
    // buf[0]=type, buf[1]=seq, buf[2..3]=steer LE, buf[4]=thr, buf[5]=brk,
    // buf[6]=flags, buf[7]=crc   (this is bytes [2..9] of the wire frame,
    // since we started copying at TYPE, not at SYNC0/SYNC1)
    uint8_t type   = buf[0];
    uint8_t seq    = buf[1];
    int16_t steer  = (int16_t)(buf[2] | (buf[3] << 8));   // little-endian
    uint8_t thr    = buf[4];
    uint8_t brk    = buf[5];
    uint8_t flags  = buf[6];
    uint8_t crc_rx = buf[7];

    if (type != CMD_TYPE) {
        last_frame_bad = true;
        return;
    }

    // CRC covers wire bytes [2..8] = our buf[0..6], i.e. type..flags (7 bytes)
    uint8_t crc_calc = link_crc8(buf, CMD_LEN - NON_DATA_BYTES);
    if (crc_calc != crc_rx) {
        last_frame_bad = true;
        return;
    }

    // reserved flags have been messed with
    if (flags & CMD_FLAG_RESERVED_MASK) {
        last_frame_bad = true;
        return;
    }

    // Frame is valid — accept it, overwrite latest (no queueing)
    unsigned int key = irq_lock(); // stop the interrupts to prevent race conditions
    latest_cmd.seq   = seq;
    latest_cmd.steer = steer;
    latest_cmd.thr   = thr;
    latest_cmd.brk   = brk;
    latest_cmd.flags = flags;
    have_fresh_cmd = true;
    last_valid_cmd_time_ms = k_uptime_get();
    ever_valid = true;
    last_frame_bad = false;
    irq_unlock(key);
}

// byte chunk handler and state machine core
static void feed_byte(uint8_t byte, int64_t now_ms)
{
    if (state != WAIT_SYNC0 && (now_ms - last_byte_time_ms) > BYTE_TIMEOUT_MS) {
        // Stalled mid-frame = truncated. Resync.
        reset_parser();
    }
    last_byte_time_ms = now_ms;

    switch (state) {
    case WAIT_SYNC0:
        if (byte == LINK_SYNC0) state = WAIT_SYNC1;
        break;

    case WAIT_SYNC1:
        if (byte == LINK_SYNC1) {
            state = WAIT_TYPE;
        } else if (byte != LINK_SYNC0) {
            reset_parser();
        }
        break;

    case WAIT_TYPE:
        // We only expect to RECEIVE commands (we're the STM32).
        // Anything else (e.g. stray STATUS_TYPE bytes, garbage) is rejected.
        if (byte != CMD_TYPE) {
            reset_parser();
            last_frame_bad = true;
            break;
        }
        buf[0] = byte;
        body_idx = 1;
        expected_len = CMD_LEN - 2;  // remaining bytes after sync0,sync1: type..crc = 8 bytes
        state = READ_BODY;
        break;

    case READ_BODY:
        buf[body_idx++] = byte;
        if (body_idx >= expected_len) {
            handle_complete_cmd_frame();
            reset_parser();
        }
        break;
    }
}

// callback for ISR
static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    uart_irq_update(dev);   // must be called once per ISR invocation — return value is void, unused

    if (!uart_irq_rx_ready(dev)) return;

    uint8_t byte;
    while (uart_fifo_read(dev, &byte, 1) == 1) {
        feed_byte(byte, k_uptime_get());
    }
}

void parser_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_comms));
    __ASSERT(device_is_ready(uart_dev), "UART device not ready");

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL); // register the UART callback function
    uart_irq_rx_enable(uart_dev);
}

bool parser_get_latest_command(cmd_frame_t *out)
{
    if (!have_fresh_cmd) return false;
    unsigned int key = irq_lock();
    *out = latest_cmd;
    have_fresh_cmd = false;
    irq_unlock(key);
    return true;
}

int64_t parser_ms_since_last_valid_command(void)
{
    if (!ever_valid) return INT64_MAX;
    return k_uptime_get() - last_valid_cmd_time_ms;
}

bool parser_last_frame_was_bad(void)
{
    return last_frame_bad;
}