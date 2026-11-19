/*
 * catalog.h - Lab 3 CAN frames (unchanged in Lab 5), plus cat_source() for every catalog frame.
 * The Raft frames and entry encoding live in shared/raft_proto.h (Aniket's, catalog v0.8).
 * Lab 3 part mirrors lab_3/car_can_app/car_can_app/src/catalog.h.
 */
#ifndef CATALOG_H
#define CATALOG_H

#include <stdint.h>
#include <stdbool.h>
#include "raft_proto.h"

#define ID_CTRL   0x010
#define ID_FAULT  0x020
#define ID_HB_N1  0x100
#define ID_HB_N2  0x101
#define ID_HB_N3  0x102
#define ID_BLINK  0x200
#define ID_FORCE  0x300

#define FAULT_TEST_BUTTON  1
#define FAULT_BAD_CTRL     2
#define FAULT_ZONE_LOST    3

#define ERR_CTRL_LOST   (1u << 0)
#define ERR_ZONE_LOST   (1u << 1)
#define ERR_TEST        (1u << 2)
#define ERR_BADCMD      (1u << 3)
#define ERR_REMOTE      (1u << 4)

static inline bool cat_is_raft(uint16_t id) { return raft_id_base(id) != 0; }

/* Sender of any catalog frame, 0 if unknown */
static inline uint8_t cat_source(uint16_t id, const uint8_t *d)
{
    if (cat_is_raft(id)) return raft_id_sender(id);
    switch (id) {
    case ID_CTRL: case ID_BLINK: case ID_HB_N1: return 1;
    case ID_HB_N2: case ID_FORCE:               return 2;
    case ID_HB_N3:                              return 3;
    case ID_FAULT:                              return d[3];
    default:                                    return 0;
    }
}

#endif /* CATALOG_H */
