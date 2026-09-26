#ifndef STATUS_TX_H
#define STATUS_TX_H

#include <stdint.h>

void status_tx_init(void);
void status_tx_send(uint8_t state, uint8_t faults,
                     uint16_t i_left_mA, uint16_t i_right_mA, uint16_t i_servo_mA);

#endif