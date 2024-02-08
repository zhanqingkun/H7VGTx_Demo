#include "debug_task.h"
#include "cmsis_os.h"
#include "data_scope.h"
#include "stdint.h"

#include "func_generator.h"
#include "us_time.h"
FGT_sin_t sin_test;

uint8_t debug_wave = 1;

void DataWavePkg(void)
{
	switch(debug_wave)
	{
		case 1:
		{
			DataScope_Get_Channel_Data(FGT_sin_Calc(&sin_test));
//			DataScope_Get_Channel_Data(imu_data.wy);
			break;
		}
		default:break;
		
	}
}

/* 串口上位机数据发送任务 */
void debug_task(void *argument)
{
	uint32_t thread_wake_time = osKernelSysTick();
	FGT_sin_Init(&sin_test, 1, 1, 1000, 5, 0);
	for(;;)
	{
		taskENTER_CRITICAL();
//		DataWave(&huart2);
		taskEXIT_CRITICAL();
		vTaskDelayUntil(&thread_wake_time, 1);
	}
}
