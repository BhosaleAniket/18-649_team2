/*
 * link_proto.h - Pi <-> STM32 UART link protocol (18-649 Lab 2, Team 2)
 *
 * Shared by the Pi bridge (bridge.c) and the STM32 firmware.
 * Copy this file into the Zephyr app UNCHANGED so both sides agree.
 *
 * UART: 115200 baud, 8N1, no flow control. Multi-byte fields little-endian.
 *
 * ---------------------------------------------------------------------------
 * COMMAND  Pi -> STM32   (10 bytes, ~0.87 ms on the wire)
 *   [0]     0xAA   sync0
 *   [1]     0x55   sync1
 *   [2]     0x01   type (CMD_TYPE)
 *   [3]     seq    uint8, +1 every frame, wraps 255 -> 0
 *   [4..5]  steer  int16   raw wheel lX: -32768 full left, 0 center, 32767 full right
 *   [6]     thr    uint8   0 released .. 255 fully pressed
 *   [7]     brk    uint8   0 released .. 255 fully pressed
 *   [8]     flags  bit0 left-blinker button held
 *                  bit1 right-blinker button held
 *                  bit2 self-test button held
 *                  bits3-7 reserved, MUST be 0
 *   [9]     CRC-8 over bytes [2..8]
 *
 *   Buttons are sent as raw LEVELS (held / not held). The STM32 does the
 *   edge detection, debounce, blinker and self-test state machines.
 *
 *   Timing: sent on every UDP wheel update, and re-sent every 20 ms when no
 *   new update arrives, so frames are never more than ~20 ms apart.
 *   If the proxy goes quiet for 100 ms the Pi STOPS sending, so the STM32
 *   sees link loss and fails safe (same as unplugging the cable).
 *   Suggested STM32 timeout: 3 missed frames x 20 ms = 60 ms
 *   (meets the 150 ms spec and the 100 ms checkoff).
 *
 * ---------------------------------------------------------------------------
 * STATUS   STM32 -> Pi   (13 bytes, every 20 ms +/-10%) = heartbeat
 *   [0]      0xAA  sync0
 *   [1]      0x55  sync1
 *   [2]      0x81  type (STATUS_TYPE)
 *   [3]      seq   uint8, +1 every frame, wraps
 *   [4]      state 0 = ERROR, 1 = NORMAL
 *   [5]      faults bit0 link lost
 *                   bit1 self-test failed
 *                   bit2 bad / out-of-range command received
 *                   bit3 power-up (no valid command yet)
 *                   bits4-7 reserved 0
 *   [6..7]   I_motor_left   uint16 mA
 *   [8..9]   I_motor_right  uint16 mA
 *   [10..11] I_servo        uint16 mA
 *   [12]     CRC-8 over bytes [2..11]
 *
 * ---------------------------------------------------------------------------
 * RECEIVER RULES (both directions)
 *   - Hunt for 0xAA 0x55 to (re)sync. A receiver that joins mid-stream or
 *     drops a byte resyncs on the next header.
 *   - Reject the frame if the type is wrong, the CRC is wrong, or a reserved
 *     bit is set. A truncated frame fails its CRC.
 *   - STM32: a rejected command counts as a bad/out-of-range command -> error state (R3).
 *
 * CRC-8: poly 0x07, init 0x00, no reflection, no final XOR (CRC-8/SMBUS).
 *        Check value: link_crc8("123456789", 9) == 0xF4
 */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>

#define LINK_SYNC0 0xAA
#define LINK_SYNC1 0x55
#define LINK_BAUD  115200

#define CMD_TYPE 0x01
#define CMD_LEN  10

#define STATUS_TYPE 0x81
#define STATUS_LEN  13

/* command flags */
#define CMD_FLAG_LEFT          (1u << 0)
#define CMD_FLAG_RIGHT         (1u << 1)
#define CMD_FLAG_TEST          (1u << 2)
#define CMD_FLAG_RESERVED_MASK 0xF8u

/* status state */
#define ST_STATE_ERROR  0
#define ST_STATE_NORMAL 1

/* status fault bits */
#define ST_FAULT_LINK     (1u << 0)
#define ST_FAULT_SELFTEST (1u << 1)
#define ST_FAULT_BADCMD   (1u << 2)
#define ST_FAULT_POWERUP  (1u << 3)
#define ST_FAULT_RESERVED_MASK 0xF0u

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