/*
Heartbeat status transmission for the link protocol
*/

#include "status_tx.h"
#include "link_proto.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

static const struct device *uart_dev;
static uint8_t tx_seq = 0;

void status_tx_init(void)
{
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_comms));
    __ASSERT(device_is_ready(uart_dev), "UART device not ready");
}

void status_tx_send(uint8_t state, uint8_t faults,
                     uint16_t i_left_mA, uint16_t i_right_mA, uint16_t i_servo_mA)
{
    uint8_t frame[STATUS_LEN]; // 13 bytes

    frame[0] = LINK_SYNC0;
    frame[1] = LINK_SYNC1;
    frame[2] = STATUS_TYPE;
    frame[3] = tx_seq++;
    frame[4] = state;
    frame[5] = faults;
    frame[6] = (uint8_t)(i_left_mA & 0xFF);
    frame[7] = (uint8_t)(i_left_mA >> 8);
    frame[8] = (uint8_t)(i_right_mA & 0xFF);
    frame[9] = (uint8_t)(i_right_mA >> 8);
    frame[10] = (uint8_t)(i_servo_mA & 0xFF);
    frame[11] = (uint8_t)(i_servo_mA >> 8);
    // CRC over wire bytes [2..11] = frame[2..11], 10 bytes
    frame[12] = link_crc8(&frame[2], STATUS_LEN - NON_DATA_BYTES);

    for (int i = 0; i < STATUS_LEN; i++) {
        uart_poll_out(uart_dev, frame[i]);
    }
}