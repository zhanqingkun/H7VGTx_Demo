#include "printf.h"

//取消ARM的半主机工作模式
#pragma import(__use_no_semihosting)
struct __FILE
{ 
	int handle; 
}; 

FILE __stdout;          
void _sys_exit(int x) 
{ 
	x = x;
}

int fputc(int ch, FILE *f)
{
	while(__HAL_UART_GET_FLAG(&DEBUG_HUART, UART_FLAG_TC) == RESET);
	HAL_UART_Transmit(&DEBUG_HUART, (uint8_t*)&ch, 1, 0xFF);
	return ch;
}

static uint8_t UartTxBuf[PRINTF_MAX_LEN];
void Dma_Printf(UART_HandleTypeDef* huart, const char *format,...)
{
	uint16_t len;
	va_list args;
	va_start(args, format);
	//如果串口对应的DMA还未发送完就等待，防止变量UartTxBuf被CPU和DMA同时使用。
	while(HAL_DMA_GetState(huart->hdmatx) == HAL_DMA_STATE_BUSY);
	len = vsnprintf((char*)UartTxBuf, sizeof(UartTxBuf)+1, (char*)format, args);
	va_end(args);
	HAL_UART_Transmit_DMA(huart, UartTxBuf, len);
}
