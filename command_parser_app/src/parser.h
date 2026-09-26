#ifndef PARSER_H
#define PARSER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t seq;
    int16_t steer;   // -32768 full left .. 32767 full right
    uint8_t thr;     // 0..255
    uint8_t brk;      // 0..255
    uint8_t flags;    // CMD_FLAG_* bits
} cmd_frame_t;

void parser_init(void);

// True if a fresh (previously-unread) valid command exists; consumes it.
bool parser_get_latest_command(cmd_frame_t *out);

// ms since last VALID command was received. INT64_MAX if none ever received.
int64_t parser_ms_since_last_valid_command(void);

// True if the most recently parsed complete frame was rejected
// (bad CRC, bad type, bad length, or reserved bits set). Sticky
// until the next fully-valid frame arrives; used to drive
// ST_FAULT_BADCMD in the status frame.
bool parser_last_frame_was_bad(void);

#endif