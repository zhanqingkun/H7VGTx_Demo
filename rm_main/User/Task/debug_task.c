#include "debug_task.h"
#include "cmsis_os.h"
#include "data_log.h"
#include "stdint.h"

#include "gimbal_task.h"
#include "shoot_task.h"
#include "wlr.h"
#include "drv_dji_motor.h"
#include "prot_judge.h"
#include "kalman_filter.h"
#include "func_generator.h"
#include "prot_dr16.h"
#include "us_time.h"

us_time_t test_time;
kalman_filter_t test;
uint8_t debug_wave = 3;

void log_scope_data_pkg(void)
{
    switch(debug_wave)
    {
        case 1://云台pid调试
        {
//            log_scope_get_data(gimbal.yaw_spd.ref);
//            log_scope_get_data(gimbal.yaw_spd.fdb);
//            log_scope_get_data(gimbal.yaw_angle.ref);
//            log_scope_get_data(gimbal.yaw_angle.fdb);
//            log_scope_get_data(gimbal.yaw_output);
//            log_scope_get_data(yaw_motor.tx_current);
            
//            log_scope_get_data(gimbal.pit_spd.ref);
//            log_scope_get_data(gimbal.pit_spd.fdb);
//            log_scope_get_data(gimbal.pit_angle.ref);
//            log_scope_get_data(gimbal.pit_angle.fdb);
//            log_scope_get_data(gimbal.pit_output);
//            log_scope_get_data(pit_motor.tx_current);
            break;
        }
        case 2://拨盘pid调试
        {
//            log_scope_get_data(shoot.trigger_spd.ref);
//            log_scope_get_data(shoot.trigger_spd.fdb);
//            log_scope_get_data(shoot.trigger_ecd.ref);
//            log_scope_get_data(shoot.trigger_ecd.fdb);
//            log_scope_get_data(shoot.trigger_output);
//            log_scope_get_data(trigger_motor.tx_current);
            break;
        }
        case 3:
        {
            log_scope_get_data(wlr.wz_set);
            log_scope_get_data(wlr.wz_fdb);
            log_scope_get_data(wlr.yaw_set);
            log_scope_get_data(wlr.yaw_fdb);
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
        log_scope_data_output();
        osDelayUntil(&thread_wake_time, 5);
    }
}
