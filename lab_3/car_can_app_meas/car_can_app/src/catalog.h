/*
 * catalog.h - Lab 3 template CAN message catalog (Team 2 payload layout)
 *
 * Shared with the Pi cockpit (cockpit.py mirrors these constants).
 * All frames: standard 11-bit ID, DLC 8, sent on bus A and bus B.
 * Byte 0: low nibble = 4-bit sequence counter (one counter per ID and
 *         sender), high nibble = flags (all 0 in this design).
 * Unused bytes are 0. Multi-byte fields little-endian.
 *
 *  ID     Name    From  Bytes 1..7
 *  0x010  CTRL    N1    [1..2] steer int16 (-32767..32767; -32768 is invalid)
 *                       [3] throttle u8  [4] brake u8  (0 released .. 255 full)
 *                       [5] buttons: bit0 left held, bit1 right held,
 *                           bit2 cockpit test held, bits3-7 must be 0
 *                       [6] CRC-8 over bytes 0..5   [7] must be 0
 *  0x020  FAULT   any   [1] failing zone id (1..3)  [2] reason (FAULT_*)
 *                       [3] reporting zone id
 *  0x100  HB_N1   N1    \  [1] zone state (0 error, 1 normal)
 *  0x101  HB_N2   N2     > [2] bus health: bit0 bus A ok, bit1 bus B ok
 *  0x102  HB_N3   N3    /  [3] error reason bits (ERR_*)  [4] zones heard (bit n = zone n)
 *                       HB_N2 only: [5..6] ms to N2's next local hazard edge (u16),
 *                       [7] N2's local hazard lamps on now (N3 aligns to it)
 *  0x200  BLINK   N1    [1] state (0 off, 1 left, 2 right, 3 hazard)
 *                       [2] phase (1 = lamps on, 0 = lamps off)
 *  0x300  FORCE   N2    [1] force int8 (dummy counter this lab)
 *
 * CRC-8: poly 0x07, init 0x00, no reflection, no final XOR (same as Lab 2).
 */
#ifndef CATALOG_H
#define CATALOG_H

#include <stdint.h>

#define ID_CTRL   0x010
#define ID_FAULT  0x020
#define ID_HB_N1  0x100
#define ID_HB_N2  0x101
#define ID_HB_N3  0x102
#define ID_BLINK  0x200
#define ID_FORCE  0x300

#define CAT_DLC   8

#define ZONE_N1   1
#define ZONE_N2   2
#define ZONE_N3   3

/* FAULT reason codes */
#define FAULT_TEST_BUTTON  1
#define FAULT_BAD_CTRL     2
#define FAULT_ZONE_LOST    3

/* Error reason bits (carried in HB byte 3) */
#define ERR_CTRL_LOST   (1u << 0)   /* no valid CTRL (power-up or proxy stopped) */
#define ERR_ZONE_LOST   (1u << 1)   /* a zone's heartbeat missing on both buses */
#define ERR_TEST        (1u << 2)   /* this zone's test button */
#define ERR_BADCMD      (1u << 3)   /* out-of-range / corrupt CTRL received */
#define ERR_REMOTE      (1u << 4)   /* another zone reported TEST or BADCMD */
/* Reasons that another zone should follow into the error state */
#define ERR_PROPAGATE   (ERR_TEST | ERR_BADCMD)

/* BLINK states */
#define BLINK_OFF     0
#define BLINK_LEFT    1
#define BLINK_RIGHT   2
#define BLINK_HAZARD  3

/* CTRL button bits */
#define CTRL_BTN_LEFT      (1u << 0)
#define CTRL_BTN_RIGHT     (1u << 1)
#define CTRL_BTN_TEST      (1u << 2)
#define CTRL_BTN_RESERVED  0xF8u

static inline uint8_t cat_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Which zone sends this frame. FAULT can come from anyone, so it carries
 * the reporter in byte 3. Returns 0 if unknown. */
static inline uint8_t cat_source(uint16_t id, const uint8_t *data)
{
    switch (id) {
    case ID_CTRL:
    case ID_BLINK:
    case ID_HB_N1: return ZONE_N1;
    case ID_HB_N2:
    case ID_FORCE: return ZONE_N2;
    case ID_HB_N3: return ZONE_N3;
    case ID_FAULT: return data[3];
    default:       return 0;
    }
}

#endif /* CATALOG_H */
