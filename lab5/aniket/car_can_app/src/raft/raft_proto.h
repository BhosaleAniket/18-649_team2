/*
 * raft_proto.h - Lab 5 Raft message catalog v0.8 (Team 2), wire constants
 *
 * Shared word for word by the Nucleos and the Pi. Any change here is a
 * catalog change: agree it with the other side and bump the version.
 *
 * Conventions: node ids 1 = cockpit (Pi), 2 = steering, 3 = drivetrain.
 * Every `bus` value is 1 = A, 2 = B; every mask uses bit (bus - 1).
 * Byte 0: low nibble = 4-bit sequence counter (filled by the CAN layer),
 *         high nibble = flags (RAFT_FLAG_*). Multi-byte fields little-endian.
 */
#ifndef RAFT_PROTO_H
#define RAFT_PROTO_H

#include <stdint.h>
#include <stdbool.h>

#define RAFT_CATALOG_VERSION "0.8"

/* ---- Node and bus ids ---- */
#define RAFT_NODE_COCKPIT   1
#define RAFT_NODE_STEERING  2
#define RAFT_NODE_DRIVE     3
#define RAFT_N_NODES        3
#define RAFT_BUS_A          1
#define RAFT_BUS_B          2
#define RAFT_BUSBIT(bus)    ((uint8_t)(1u << ((bus) - 1)))
#define RAFT_NODEBIT(n)     ((uint8_t)(1u << ((n) - 1)))

/* ---- CAN IDs: base + sender id (1..3); sender = id & 0x07 ---- */
#define RAFT_ID_REF         0x008   /* 0x009..0x00B leader, every heartbeat_ms */
#define RAFT_ID_REQ_VOTE    0x030
#define RAFT_ID_VOTE        0x040
#define RAFT_ID_APPEND      0x050
#define RAFT_ID_ACK         0x060
#define RAFT_ID_REPORT      0x070
#define RAFT_ID_REDIRECT    0x080

static inline uint8_t raft_id_sender(uint16_t id) { return (uint8_t)(id & 0x07); }

/* Base of a Raft ID, or 0 if the ID is not a Raft frame */
static inline uint16_t raft_id_base(uint16_t id)
{
    uint8_t s = (uint8_t)(id & 0x07);
    if (s < 1 || s > 3) {
        return 0;
    }
    if ((id & 0x7F8) == RAFT_ID_REF) {
        return RAFT_ID_REF;
    }
    if ((id & 0x008) != 0) {
        return 0;
    }
    uint16_t b = (uint16_t)(id & 0x7F0);
    if (b >= RAFT_ID_REQ_VOTE && b <= RAFT_ID_REDIRECT) {
        return b;
    }
    return 0;
}

/* ---- Byte 0 flags (high nibble) ---- */
#define RAFT_FLAG_REPLAYED  0x10    /* entry re-sent during catch-up */
#define RAFT_FLAG_RELAYED   0x20    /* set by the node that forwarded the frame (Raft frames only) */

/*
 * Frame layouts, bytes 1..7 (see the catalog doc):
 *   REF      [1] term  [2] cycle u8  [3] bits 1..0 stamp bus (0 = none), bits 7..2 = bits 37..32
 *            of the stamp  [4..7] stamp bits 31..0: TX-complete time, in global us, of the
 *            PREVIOUS REF (cycle - 1) on that bus. 38 bits = 76 h before wrapping.
 *   REQ_VOTE [1] term  [2..3] last log index  [4] last log term  [5] pre-vote (1 = ask only)
 *   VOTE     [1] term  [2] candidate  [3] granted  [4] pre-vote echo
 *   APPEND   [1] term  [2..3] prev index  [4] prev term  [5] entry (0 = none)
 *            [6] entry term  [7] commit back: commit = ref - back, ref = entry index
 *            (or prev index when empty); 255 = no commit information
 *   ACK      [1] term  [2..3] match index (failure: hint = follower's last index)  [4] success  [5] leader id
 *   REPORT   [1] entry  [2..3] report id  [4] leader the client believes in
 *   REDIRECT [1] current leader (0 = unknown)  [2..3] report id echoed  [4] reporter
 */
#define RAFT_COMMIT_BACK_NONE 255

/* ---- Entry byte ----
 * bits 7..5 type, bits 4..3 node (1..3), bit 2 bus (0 = A, 1 = B),
 * bit 1 = 0, bit 0 = up (1) / down (0). APP_EVENT: bits 2..0 = code. */
#define RAFT_ET_NONE        0
#define RAFT_ET_NOP         1
#define RAFT_ET_COCKPIT     2
#define RAFT_ET_NODE        3
#define RAFT_ET_LINK        4
#define RAFT_ET_EVENT       5

#define RAFT_ENT_TYPE(e)    ((uint8_t)((e) >> 5))
#define RAFT_ENT_NODE(e)    ((uint8_t)(((e) >> 3) & 0x03))
#define RAFT_ENT_BUS(e)     ((uint8_t)((((e) >> 2) & 0x01) + 1))
#define RAFT_ENT_UP(e)      ((bool)((e) & 0x01))
#define RAFT_ENT_CODE(e)    ((uint8_t)((e) & 0x07))

#define RAFT_ENT_MAKE_NOP()             ((uint8_t)0x20)
#define RAFT_ENT_MAKE_COCKPIT(ok)       ((uint8_t)(0x40 | ((ok) ? 1 : 0)))
#define RAFT_ENT_MAKE_NODE(n, up)       ((uint8_t)(0x60 | ((n) << 3) | ((up) ? 1 : 0)))
#define RAFT_ENT_MAKE_LINK(n, bus, up)  ((uint8_t)(0x80 | ((n) << 3) | (((bus) - 1) << 2) | ((up) ? 1 : 0)))
#define RAFT_ENT_MAKE_EVENT(o, code)    ((uint8_t)(0xA0 | ((o) << 3) | ((code) & 0x07)))

/* MEMBER class = types 1..4, EVENT class = type 5 */
static inline bool raft_ent_is_member(uint8_t e)
{
    uint8_t t = RAFT_ENT_TYPE(e);
    return t >= RAFT_ET_NOP && t <= RAFT_ET_LINK;
}

/* APP_EVENT codes */
#define RAFT_EVT_TEST_BUTTON 1

/* ---- Little-endian helpers ---- */
static inline uint16_t raft_get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void raft_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline uint32_t raft_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void raft_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* 8-bit terms compared as serial numbers: a newer than b */
static inline bool raft_term_newer(uint8_t a, uint8_t b) { return (int8_t)(uint8_t)(a - b) > 0; }

/* Text for console logs: "MEMBER LINK steering A DOWN", "EVENT steering 1" */
const char *raft_node_name(uint8_t n);
int raft_entry_str(uint8_t e, char *buf, int len);

#endif /* RAFT_PROTO_H */
