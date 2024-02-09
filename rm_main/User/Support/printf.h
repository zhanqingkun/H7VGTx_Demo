//使用方法：3种 printf() Dma_Printf() DEBUG()
//	0. 重定向fpuc()来实现标准printf()，没有DMA功能，不推荐使用
//
//	0. 使用较底层的Dma_Printf，可以自由选择串口，如Dma_Printf(&huart,"hello %s\n","HZT");
//	1. 有DMA功能
//
//	0. 首先在control_def.h定义好使用的串口方式和对应串口
//	1. 使用DEBUG()来打印，和printf()使用方式相同
//
//修改本文件的PRINTF_MAX_LEN来决定发送内容最大长度
//推荐使用DEBUG()，因为可以通过宏定义__DEBUG__为0来取消所有DEBUG()

#ifndef __PRINTF__H__
#define __PRINTF__H__

#include "stdio.h"
#include "stdarg.h"
#include "usart.h"

#define DEBUG_HUART huart2
#define PRINTF_MAX_LEN 100
#define __DEBUG__ 0

#if __DEBUG__ == 1
	#define DEBUG(format, ...) Dma_Printf(&DEBUG_HUART, format, ##__VA_ARGS__)
#elif __DEBUG__ == 2
	#define DEBUG(format, ...) Dma_Printf(&DEBUG_HUART, "FILE:"__FILE__",LINE:%d : "format"/n", __LINE__, ##__VA_ARGS__)
	//第二条的打印内容会加上调用DEBUG的文件名和行数，方便定位或者用于断言函数
#elif __DEBUG__ == 0
	#define DEBUG(format, ...)
#endif

int fputc(int ch, FILE *f);
void Dma_Printf(UART_HandleTypeDef* huart, const char *format, ...);

#endif
