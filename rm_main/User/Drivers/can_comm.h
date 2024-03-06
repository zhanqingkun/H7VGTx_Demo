#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32h7xx.h"

void can_comm_init(void);
void can1_std_transmit(uint32_t id, uint8_t *data);
void can2_std_transmit(uint32_t id, uint8_t *data);
void can3_std_transmit(uint32_t id, uint8_t *data);

#endif
