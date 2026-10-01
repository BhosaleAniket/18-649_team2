#ifndef CURRENT_H
#define CURRENT_H

#include <stdbool.h>
#include <stdint.h>

/* Averaged current magnitudes in mA, for the status frame. */
struct current_readings {
    uint16_t left_ma;
    uint16_t right_ma;
    uint16_t servo_ma;
};

int current_init(void);                         /* starts the sampling thread */
void current_get(struct current_readings *out); /* latest averaged values */
bool current_is_valid(void);                    /* boot zero found in range */

#endif
