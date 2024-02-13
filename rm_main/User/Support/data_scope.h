//使用方法：
//  0. 选择所用串口
//  1. 在 data_scope.h 选择使用的上位机，根据需要修改示波器通道数
//     其中，若使用 MiniBalance，则最大通道数默认10
//  2. 改写 __weak void DataWavePkg(void) 函数，在其中加入待发送的数据
//     void DataWavePkg(void)
//     {
//          the data needed to send:
//          DataScope_Get_Channel_Data(float_type_data1);
//          DataScope_Get_Channel_Data(float_type_data2);
//     }
//  3. 周期执行 DataWave()
//     函数即可一次性将注册数据当前值顺序发出

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "usart.h"
#include "stdio.h"

//DATA_DEBUG_LEVEL为0U时，取消各种DEBUG
//                为1U时，使用Debug_Log打印日志
//                为2U时，为VOFA+打印波形
//                为3U时，为MiniBalance打印波形
#define DATA_DEBUG_MODE 2U
#define DATA_MAX_NUM    16                    //VOFA+示波器最大通道数
#define DATA_DEBUG_UART huart2                //使用data_scope文件中各种DEBUG函数时所用串口
#define DATA_DEBUG_LEN  100                   //使用DMA_printf时字符串最大长度

#if (DATA_DEBUG_MODE == 1U)
    #define Debug_Log(format, ...) DMA_printf("FILE:" __FILE__ \
                                   ",LINE:%d :" format "/n", __LINE__, ##__VA_ARGS__)
#else
    #define Debug_Log(...)
#endif

int fputc(int ch, FILE *f);
void DMA_printf(const char *format, ...);

void DataWavePkg(void);                         //重写此弱函数
void DataScope_Get_Channel_Data(float Data);    //按顺序注册待打印数据
void DataWave(void);                            //一次性按顺序将注册数据打印到上位机

#ifdef __cplusplus
}
#endif

#endif
