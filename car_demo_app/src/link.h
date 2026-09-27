#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/* Latest valid command from the Pi. */
struct link_cmd {
    bool     valid;     /* at least one valid frame received since boot */
    uint8_t  seq;
    int16_t  steer;     /* -32768 full left .. 32767 full right */
    uint8_t  thr;       /* 0 released .. 255 full */
    uint8_t  brk;       /* 0 released .. 255 full */
    uint8_t  flags;     /* CMD_FLAG_* levels */
    int64_t  rx_ms;     /* uptime when it arrived */
};

/* Given by the RX ISR on every valid command; the control thread waits on it. */
extern struct k_sem link_cmd_sem;

int link_init(void);                        /* also starts the status thread */
void link_get_command(struct link_cmd *out); /* consistent snapshot */
uint32_t link_bad_frames(void);             /* total rejected frames since boot */

#endif