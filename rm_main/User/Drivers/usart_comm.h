#ifndef __USART_COMM_H
#define __USART_COMM_H

#include "stm32h7xx.h"
#include "usart.h"
#include "string.h"

//串口定义
#define	DBUS_HUART	huart1
#define DEBUG_HUART	huart2

void USART_Comm_Init(void);
void USART_User_IRQHandler(UART_HandleTypeDef *huart);

#endif
