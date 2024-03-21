#include "gimbal_task.h"
#include "drv_dji_motor.h"
#include "prot_dr16.h"
#include "prot_imu.h"
#include "math_lib.h"
#include "cmsis_os.h"
#include "string.h"

gimbal_t gimbal;

static void gimbal_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
    pid_init(&gimbal.yaw_angle.pid, 0, 0, 0, 0, 0);
    pid_init(&gimbal.yaw_spd.pid, 0, 0, 0, 0, 0);
    pid_init(&gimbal.yaw_mecd.pid, 0, 0, 0, 0, 0);
    pid_init(&gimbal.yaw_mspd.pid, 0, 0, 0, 0, 0);

    pid_init(&gimbal.pit_angle.pid, 0, 0, 0, 0, 0);
    pid_init(&gimbal.pit_spd.pid, 0, 0, 0, 0, 0);
    pid_init(&gimbal.pit_mecd.pid, 0, 0, 0, 0, 0);
    pid_init(&gimbal.pit_mspd.pid, 0, 0, 0, 0, 0);

    scale.ch1 = 2;
    scale.ch2 = 2;
}

static void gimbal_data_input(void)
{
    
}

static void gimbal_calc(void)
{
    gimbal.pit_angle.fdb = gimbal_imu.pit;
    pid_calc(&gimbal.pit_angle.pid, gimbal.pit_angle.ref, gimbal.pit_angle.fdb);
    gimbal.pit_spd.ref = gimbal.pit_angle.pid.output;
    gimbal.pit_spd.fdb = gimbal_imu.wy;
    pid_calc(&gimbal.pit_spd.pid, gimbal.pit_spd.ref, gimbal.pit_spd.fdb);

    gimbal.yaw_angle.fdb = gimbal_imu.yaw;
    pid_calc(&gimbal.yaw_angle.pid, gimbal.yaw_angle.ref, gimbal.yaw_angle.fdb);
    gimbal.yaw_spd.ref = gimbal.yaw_angle.pid.output;
    gimbal.yaw_spd.fdb = gimbal_imu.wz;
    pid_calc(&gimbal.yaw_spd.pid, gimbal.yaw_spd.ref, gimbal.yaw_spd.fdb);
}

static void gimbal_data_output(void)
{
//    dji_motor_set_torque(&pit_motor, gimbal.pit_mspd.pid.output);
//    dji_motor_set_torque(&yaw_motor, gimbal.yaw_mspd.pid.output);

    dji_motor_set_torque(&pit_motor, gimbal.pit_spd.pid.output);
    dji_motor_set_torque(&yaw_motor, gimbal.yaw_spd.pid.output);
}

void gimbal_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    gimbal_init();
    for(;;)
    {
        gimbal_data_input();
        gimbal_calc();
        gimbal_data_output();
        osDelayUntil(&thread_wake_time, 2);
    }
}
