#include "debug_task.h"
#include "cmsis_os.h"
#include "data_scope.h"
#include "stdint.h"

#include "func_generator.h"
#include "us_time.h"

us_time_t test_time;
uint8_t debug_wave = 1;

void DataWavePkg(void)
{
    switch(debug_wave)
    {
        case 1:
        {
            usTime_Interval_Test_Start(&test_time);
            usTime_Delay(996);
//            usTime_Delay(200);
            usTime_Interval_Test_End(&test_time);
           DataScope_Get_Channel_Data(test_time.dt);
//            DataScope_Get_Channel_Data(usTime_Period_Test(&test_time));
            break;
        }
        default:break;
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void *argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        thread_wake_time += 100;
//        taskENTER_CRITICAL();
        DataWave();
//        taskEXIT_CRITICAL();
        osDelayUntil(thread_wake_time);
    }
}
