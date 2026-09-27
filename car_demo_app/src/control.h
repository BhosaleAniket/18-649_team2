#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

struct control_telemetry {
    int rpm_x10;        /* averaged, filtered wheel speed */
    int target_x10;     /* commanded wheel speed */
    int duty_pct;       /* motor EN duty */
    int braking;        /* 1 while dynamic braking */
};

int control_init(void);                 /* starts the control thread */
uint8_t control_get_state(void);        /* ST_STATE_NORMAL / ST_STATE_ERROR */
uint8_t control_get_faults(void);       /* ST_FAULT_* bits */
void control_get_telemetry(struct control_telemetry *out);

#endif