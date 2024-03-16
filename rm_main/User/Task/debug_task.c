#include "debug_task.h"
#include "cmsis_os.h"
#include "data_log.h"
#include "stdint.h"

#include "func_generator.h"
#include "us_time.h"

us_time_t test_time;
uint8_t debug_wave = 1;

void log_scope_data_pkg(void)
{
    switch(debug_wave)
    {
        case 1:
        {
//            us_timer_interval_test_start(&test_time);
//            us_timer_delay(200);
//            us_timer_interval_test_end(&test_time);
//            log_scope_get_data(test_time.dt);
            log_scope_get_data(2);
            break;
        }
        default:break;
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
//        taskENTER_CRITICAL();
        log_scope_data_output();
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 10);
    }
}
