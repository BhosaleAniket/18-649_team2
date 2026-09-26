#include <zephyr/kernel.h>
#include "parser.h"
#include "status_tx.h"
#include "link_proto.h"

#define FAIL_SAFE_MS 60   // per link_proto.h note: 3 x 20ms, meets 150ms spec, TODO: may need to adjust for delay bounds

static uint8_t current_faults = ST_FAULT_POWERUP;
static uint8_t current_state = ST_STATE_ERROR;

static void apply_command(const cmd_frame_t *cmd)
{
    printk("steer=%d thr=%u brk=%u flags=0x%02x seq=%u\n",
           cmd->steer, cmd->thr, cmd->brk, cmd->flags, cmd->seq);
    // TODO: your motor/servo driver calls go here
}

static void enter_fail_safe(void)
{
    printk("FAIL SAFE (link lost)\n");
    current_state = ST_STATE_ERROR;
    current_faults |= ST_FAULT_LINK;
    // TODO: zero throttle/brake-on, safe steering here
}

int main(void)
{
    parser_init();
    status_tx_init();

    int64_t next_status_tx = k_uptime_get();

    while (1) {
        int64_t now = k_uptime_get();

        // --- Command side ---
        if (parser_ms_since_last_valid_command() >= FAIL_SAFE_MS) {
            enter_fail_safe();
        } else {
            cmd_frame_t cmd;
            if (parser_get_latest_command(&cmd)) {
                current_faults &= ~(ST_FAULT_LINK | ST_FAULT_POWERUP);
                if (parser_last_frame_was_bad()) {
                    current_faults |= ST_FAULT_BADCMD;
                    current_state = ST_STATE_ERROR;
                } else {
                    current_faults &= ~ST_FAULT_BADCMD;
                    current_state = ST_STATE_NORMAL;
                }
                apply_command(&cmd);
            }
        }

        // --- Status side: send every 20ms ---
        if (now >= next_status_tx) {
            uint16_t i_left = 0, i_right = 0, i_servo = 0; // TODO: read real ADC values
            status_tx_send(current_state, current_faults, i_left, i_right, i_servo);
            next_status_tx += 20;
        }

        k_msleep(1);
    }
}