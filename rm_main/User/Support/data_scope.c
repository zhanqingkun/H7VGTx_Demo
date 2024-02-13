#include "data_scope.h"
#include "stdarg.h"

//<-------------------------------------------printf重定向------------------------------------------->
//printf重定向使用的为HAL_UART_Transmit，
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
	while(__HAL_UART_GET_FLAG(&DATA_DEBUG_UART, UART_FLAG_TC) == RESET);
	HAL_UART_Transmit(&DATA_DEBUG_UART, (uint8_t*)&ch, 1, 0xFF);
	return ch;
}

//<-------------------------------------------DMA_printf------------------------------------------->
static uint8_t UartTxBuf[DATA_DEBUG_LEN];
void DMA_printf(const char *format, ...)
{
	uint16_t len;
	va_list args;
	va_start(args, format);
	//如果串口对应的DMA还未发送完就等待，防止变量UartTxBuf被CPU和DMA同时使用。
	while(HAL_DMA_GetState(DATA_DEBUG_UART.hdmatx) == HAL_DMA_STATE_BUSY);
	len = vsnprintf((char*)UartTxBuf, sizeof(UartTxBuf)+1, (char*)format, args);
	va_end(args);
	HAL_UART_Transmit_DMA(&DATA_DEBUG_UART, UartTxBuf, len);
}

//<-------------------------------------------DataScope------------------------------------------->
//示波器数据结构体
static struct _DataTypedfef_t
{
    unsigned char OutPut_Buffer[4 * DATA_MAX_NUM + 4]; //串口发送缓冲区
    unsigned char Send_Count;                          //串口需要发送的数据个数
    unsigned char Data_Num;                            //变量数量
} CK;

//@breif  将单精度浮点数据转成4字节数据并存入指定地址
//@param  target: 目标浮点数据
//@param  buf   : 目标地址
//@param  offset: 地址偏置
//@retval None
static void Float2Byte(float *target, unsigned char *buf, unsigned char offset)
{
    unsigned char *point;
    point = (unsigned char*)target;//得到float的地址
    buf[offset]   = point[0];
    buf[offset+1] = point[1];
    buf[offset+2] = point[2];
    buf[offset+3] = point[3];
}

//@breif  将待发送通道的单精度浮点数据写入发送缓冲区
//@param  Data: 目标浮点数据
//@retval None
void DataScope_Get_Channel_Data(float Data)
{
#if (DATA_DEBUG_MODE == 2U)//VOFA+
    if(CK.Data_Num >= DATA_MAX_NUM)
        return;
    else
    {
        CK.Data_Num++;
        Float2Byte(&Data, CK.OutPut_Buffer, ((CK.Data_Num - 1) * 4));
    }
#elif (DATA_DEBUG_MODE == 3U)//MINIBALANCE
    if(CK.Data_Num >= 10)//MINIBALANCE最多10个通道
        return;
    else
    {
        CK.Data_Num++;
        Float2Byte(&Data, CK.OutPut_Buffer, ((CK.Data_Num - 1) * 4 + 1));//留出帧头
    }
#else
    return;
#endif
}

//@breif  生成能正确识别的帧格式
//@param  Channel_Number: 需要发送的通道个数
//@retval 返回发送缓冲区数据个数
static unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if(Channel_Number == 0)
        return 0;
    else
    {
#if (DATA_DEBUG_MODE == 2U)//VOFA+
        uint8_t temp_cnt = Channel_Number * 4 + 4;
        CK.OutPut_Buffer[4 * Channel_Number + 0] = 0x00;
        CK.OutPut_Buffer[4 * Channel_Number + 1] = 0x00;
        CK.OutPut_Buffer[4 * Channel_Number + 2] = 0x80;
        CK.OutPut_Buffer[4 * Channel_Number + 3] = 0x7f;
        return temp_cnt;//返回一个数据包的字节数
#elif (DATA_DEBUG_MODE == 3U)//MINIBALANCE
        CK.OutPut_Buffer[0] = '$';//帧头
        uint8_t temp_cnt = Channel_Number * 4 + 1;
        CK.OutPut_Buffer[temp_cnt]  =  temp_cnt;//帧尾
        return (temp_cnt+1);//返回一个数据包的字节数
#else
        return 0;
#endif
    }
}

//@breif 重写此函数，用来注册需要发送的数据
__weak void DataWavePkg(void)
{
    //注册格式如下
    //DataScope_Get_Channel_Data(float_type_data1);
    //DataScope_Get_Channel_Data(float_type_data2);
}

//@breif  上位机通过串口打印数据波形
//@retval None
//@note   周期调用此函数
void DataWave(void)
{
    DataWavePkg();
    CK.Send_Count = DataScope_Data_Generate(CK.Data_Num);
    if(CK.Send_Count != 0)
        HAL_UART_Transmit_DMA(&DATA_DEBUG_UART, CK.OutPut_Buffer, CK.Send_Count);
    CK.Data_Num = 0;
    CK.Send_Count = 0;
}
