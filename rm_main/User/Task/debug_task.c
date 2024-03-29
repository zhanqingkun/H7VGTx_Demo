#include "debug_task.h"
#include "cmsis_os.h"
#include "data_log.h"
#include "stdint.h"

#include "prot_judge.h"
#include "kalman_filter.h"
#include "func_generator.h"
#include "prot_dr16.h"
#include "us_time.h"

us_time_t test_time;
kalman_filter_t test;
uint8_t debug_wave = 1;

void log_scope_data_pkg(void)
{
    switch(debug_wave)
    {
        case 1:
        {
            log_scope_get_data(frame_header.seq);
//            us_timer_interval_test_start(&test_time);
//            us_timer_delay(200);
//            us_timer_interval_test_end(&test_time);
//            log_scope_get_data(test_time.dt);
//            test.measured_vector[0] = (float)rc.ch1;
//            test.measured_vector[1] = (float)rc.ch2;
//            kalman_filter_update(&test);
//            log_scope_get_data(test.filter_vector[0]);
//            log_scope_get_data((float)rc.ch1);
//            log_scope_get_data((float)rc.ch2);
            break;
        }
        default:break;
    }
}

/* 串口上位机数据发送任务 */
void debug_task(void const* argument)
{
    kalman_filter_init(&test, 1, 0, 2);
    test.A_data[0] = 1;
    test.H_data[0] = 1;
    test.H_data[1] = 1;
    test.Q_data[0] = 1;
    test.R_data[0] = 100;
    test.R_data[3] = 100;
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        log_scope_data_output();
        osDelayUntil(&thread_wake_time, 5);
    }
}
