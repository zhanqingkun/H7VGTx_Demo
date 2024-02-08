//MiniBalance 和 VOFA+ 串口上位机驱动程序
//使用方法：
//	0. 默认使用F4系列，否则需要修改DataWave()中的串口发送寄存器
//	1. 在 data_scope.h 选择使用的上位机，根据需要修改示波器通道数
//		其中，若使用 MiniBalance，则最大通道数默认10
//	2. 改写 __weak void DataWavePkg(void) 函数，在其中加入待发送的数据
//		void DataWavePkg(void)
//		{
//			the data needed to send:
//			DataScope_Get_Channel_Data(float_type_data1);
//			DataScope_Get_Channel_Data(float_type_data2);
//		}
//	3. 周期执行 DataWave(UART_HandleTypeDef* huart)
//		函数即可一次性将注册数据当前值顺序发出
//
//	若使用 MiniBalance 上位机：或需要自行分频发送，避免卡机
//	若使用 VOFA+ 上位机：上位机中需要使用 Justfloat 协议
#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "usart.h"

//#define MINIBALANCE       //若要使用VOFA+串口上位机，则注释此宏
#define MAX_SEND_NUM 10     //MINIBALANCE最多10个通道，VOFA+可以超过10个通道

//定义示波器通道数
#ifndef MINIBALANCE
	#define DATASCOPE_MAX_SEND_NUM MAX_SEND_NUM //使用用户定义的最大通道数
#else
	#define DATASCOPE_MAX_SEND_NUM 10 //默认10个通道
#endif

/* 示波器数据结构体 */
static struct _DataTypedfef_t
{
	unsigned char OutPut_Buffer[4*DATASCOPE_MAX_SEND_NUM+4];//串口发送缓冲区
	unsigned char Send_Count;//串口需要发送的数据个数
	unsigned char Data_Num;//变量数量
} CK;

void DataWavePkg(void);                         //重写此弱函数
void DataScope_Get_Channel_Data(float Data);    //按顺序注册待打印数据
void DataWave(UART_HandleTypeDef* huart);       //一次性按顺序将注册数据打印到上位机

#endif
