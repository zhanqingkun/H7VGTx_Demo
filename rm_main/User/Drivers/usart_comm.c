#include "usart_comm.h"
#include "DT7control_driver.h"

#define DEBUG_DATA_LEN 10

uint8_t dbus_dma_rx_buf[2*DT7_DATA_LEN];
uint8_t debug_dma_rx_buf[2*DEBUG_DATA_LEN];

//串口初始化
//开启空闲中断并开始DMA接收数据
void USART_Comm_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&DBUS_HUART, dbus_dma_rx_buf, 2*DT7_DATA_LEN);
	
	__HAL_UART_CLEAR_IDLEFLAG(&DEBUG_HUART);
	__HAL_UART_ENABLE_IT(&DEBUG_HUART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&DEBUG_HUART, debug_dma_rx_buf, 2*DEBUG_DATA_LEN);
}

//全满中断 接收数据在buff后半段
//调用数据处理函数，若数据错误，开启空闲中断，重新对齐数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &DBUS_HUART)
	{
		if(DT7control_Receive(&rc, &dbus_dma_rx_buf[DT7_DATA_LEN]))
		{
			__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
			__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
		}
	}
	else if(huart == &DEBUG_HUART)
	{
		HAL_UART_Transmit_DMA(huart, &debug_dma_rx_buf[DEBUG_DATA_LEN], DEBUG_DATA_LEN);
	}
}

//半满中断 接收数据在buff前半段
//调用数据处理函数，若数据错误，开启空闲中断，重新对齐数据
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &DBUS_HUART)
	{
		if(DT7control_Receive(&rc, &dbus_dma_rx_buf[0]))
		{
			__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
			__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
		}
	}
	else if(huart == &DEBUG_HUART)
	{
		HAL_UART_Transmit_DMA(huart, &debug_dma_rx_buf[0], DEBUG_DATA_LEN);
	}
}

//串口中断
//进入空闲中断后重新进行DMA数据接收，然后关闭空闲中断
//用于将一组数据对齐
void USART_User_IRQHandler(UART_HandleTypeDef *huart)
{
	//空闲中断一次 用来多字节数据对齐
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
	{
		//一帧数据接收完成
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_DMAStop(huart);
		if(huart == &DBUS_HUART)
		{
			__HAL_UART_DISABLE_IT(&DBUS_HUART, UART_IT_IDLE);
			HAL_UART_Receive_DMA(huart, dbus_dma_rx_buf, 2*DT7_DATA_LEN);
		}
		else if(huart == &DEBUG_HUART)
		{
			__HAL_UART_DISABLE_IT(&DEBUG_HUART, UART_IT_IDLE);
			HAL_UART_Receive_DMA(huart, debug_dma_rx_buf, 2*DEBUG_DATA_LEN);
		}
	}
}

//备注：当以一组数据（多个uint8_t）进行通信时，有可能出现数据没有对齐的情况
//		简单来说就是，单片机刚上电时接收到的第一个数据可能是外界发送一组数据里内的任意一个，而不是这一组数据里的第一个
//		这时如果没有开空闲中断进行处理，数据就会一直错位
//		本文件中的空闲中断处理用于数据对齐，经测试上电后只需进行一次即可，后续数据将一直保持对齐
//		而在半满中断回调函数、全满中断回调函数中再添加空闲中断只是为了保险起见
