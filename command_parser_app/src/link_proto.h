/*
Packet definition for the link protocol

CMD Packet
[0]     0xAA   sync0
[1]     0x55   sync1
[2]     0x01   type
[3]     seq_counter    uint8
[4..5]  steer  int16, little-endian
[6]     thr    uint8
[7]     brk    uint8
[8]     flags  bitfield
[9]     CRC-8 over bytes [2..8]

Heartbeat Status Packet
[0..1]   sync
[2]      0x81 type
[3]      seq_counter
[4]      state (0=ERROR, 1=NORMAL)
[5]      faults (bitfield)
[6..7]   I_motor_left, uint16 mA, little-endian
[8..9]   I_motor_right
[10..11] I_servo
[12]     CRC-8 over bytes [2..11]
*/


#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>

// distinct bytes to let the receiver synchronize with the incoming data stream
#define LINK_SYNC0 0xAA
#define LINK_SYNC1 0x55

// UART baud rate
#define LINK_BAUD  115200

#define CMD_TYPE 0x01 // PI to STM32 command
#define CMD_LEN  10 // length of the command packet

#define STATUS_TYPE 0x81 // heartbeat status command
#define STATUS_LEN  13 // length of the status packet

#define CMD_FLAG_LEFT          (1u << 0)
#define CMD_FLAG_RIGHT         (1u << 1)
#define CMD_FLAG_TEST          (1u << 2)
#define CMD_FLAG_RESERVED_MASK 0xF8u // not using these right now

#define NON_DATA_BYTES 3 // number of bytes that are not data (sync0, sync1, CRC)

#define ST_STATE_ERROR  0
#define ST_STATE_NORMAL 1

#define ST_FAULT_LINK     (1u << 0)
#define ST_FAULT_SELFTEST (1u << 1)
#define ST_FAULT_BADCMD   (1u << 2)
#define ST_FAULT_POWERUP  (1u << 3)
#define ST_FAULT_RESERVED_MASK 0xF0u // not using these right now

// standard CRC function
static inline uint8_t link_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

#endif /* LINK_PROTO_H */