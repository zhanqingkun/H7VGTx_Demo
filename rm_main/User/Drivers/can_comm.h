#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32h7xx_hal.h"
#include "fdcan.h"

void CAN_Comm_Init(void);
void CAN1_Transmit(uint32_t tx_id, uint8_t* tx_data);
void CAN2_Transmit(uint32_t tx_id, uint8_t* tx_data);
void CAN3_Transmit(uint32_t tx_id, uint8_t* tx_data);

#endif
