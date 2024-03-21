#include "chassis_task.h"
#include "mode_switch_task.h"
#include "wlr.h"
#include "can_comm.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_imu.h"
#include "prot_dr16.h"
#include "pid.h"
#include "math_lib.h"
#include "cmsis_os.h"

#define JOINT_MOTOR_RESET_TORQUE 1.5f
#define JOINT_MOTOR_RESET_ERROR 0.005f

chassis_t chassis;
pid_t pid_yaw;

static void joint_motor_reset(void)
{
    static motor_reset_t motor_reset[4];
    uint32_t thread_wake_time = osKernelSysTick();
    //清除上次复位数据和电机数据
    dji_motor_set_torque(&driver_motor[0], 0);
    dji_motor_set_torque(&driver_motor[1], 0);
    chassis.joint_motor_reset = 0;
    for (int i = 0; i < 4; i++) {
        motor_reset[i].reset_flag = 0;
        ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
    }
    //未解锁 等待
    while (!mode.lock_flag)
        osDelayUntil(&thread_wake_time, 2);
    //开始复位
    while (!chassis.joint_motor_reset) {
        if (motor_reset[0].reset_flag == 1 && motor_reset[1].reset_flag == 1 && \
            motor_reset[2].reset_flag == 1 && motor_reset[3].reset_flag == 1)
            chassis.joint_motor_reset = 1;
        else
            chassis.joint_motor_reset = 0;
        for (int i = 0; i < 4; i++) {
            if (motor_reset[i].reset_flag == 0) {
                if (i == 0 || i == 2)
                    ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, -JOINT_MOTOR_RESET_TORQUE);
                else
                    ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, JOINT_MOTOR_RESET_TORQUE);
                if (fabs(motor_reset[i].last_position - joint_motor[i].position) < JOINT_MOTOR_RESET_ERROR)
                    motor_reset[i].stop_cnt++;
                else
                    motor_reset[i].stop_cnt = 0;
                if (motor_reset[i].stop_cnt >= 100) {
                    motor_reset[i].reset_flag = 1;
                    ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
                    ht_motor_set_control_cmd(&joint_motor[i], CMD_ZERO_POSITION);
                } else {
                    motor_reset[i].last_position = joint_motor[i].position;
                }
            } else {
                ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
            }
        }
        osDelayUntil(&thread_wake_time, 2);
    }
    mode.chassis_flag.reset = 0;
}

static void chassis_init()
{
    pid_init(&pid_yaw, 5, 0, 0, 0, 5);
    wlr_init();
	joint_motor_reset();
    wlr.yaw_set = chassis_imu.yaw;
}

static void chassis_data_input(void)
{
    //模式输入
    wlr.ctrl_mode = mode.chassis_flag.ctrl;
    wlr.high_flag = mode.chassis_flag.hight;
    wlr.stop_flag = mode.chassis_flag.stop;
    if (wlr.jump_flag == 0 && mode.chassis_flag.jump == 1)
        wlr.jump_flag = 1;
    //控制数据输入
    if (mode.ctrl_mode == PROTECT_MODE) {
		wlr.yaw_set = chassis_imu.yaw;
		wlr.wz_set = 0;
	}
    wlr.yaw_set -= (float)rc.ch1/660/100;
    if (wlr.yaw_set < 0)
        wlr.yaw_set += 2 * PI;
    else if (wlr.yaw_set > 2 * PI)
        wlr.yaw_set -= 2 * PI;
	wlr.wz_set = pid_calc(&pid_yaw, wlr.yaw_set, wlr.yaw_set - circle_error(&wlr.yaw_set, &chassis_imu.yaw, 2 * PI));
	wlr.v_set = (float)rc.ch4/660*3;
    //陀螺仪数据输入
    wlr.roll_fdb    = -chassis_imu.rol;
    wlr.pit_fdb     =  chassis_imu.pit;
    wlr.wy_fdb      =  chassis_imu.wy;
    wlr.wz_fdb      =  chassis_imu.wz;
    wlr.az_fdb      =  chassis_imu.az;
    //电机数据输入
    wlr.side[0].q4 =  joint_motor[0].position - joint_motor[0].zero_point;
    wlr.side[0].q1 =  joint_motor[1].position + joint_motor[1].zero_point + PI;
    wlr.side[0].w4 =  joint_motor[0].velocity;
    wlr.side[0].w1 =  joint_motor[1].velocity;
    wlr.side[0].t4 =  joint_motor[0].torque;
    wlr.side[0].t1 =  joint_motor[1].torque;
    wlr.side[0].wy = -driver_motor[0].velocity;//可能需要加滤波
    
    wlr.side[1].q1 = -joint_motor[2].position + joint_motor[2].zero_point + PI;
    wlr.side[1].q4 = -joint_motor[3].position - joint_motor[3].zero_point;
    wlr.side[1].w1 = -joint_motor[2].velocity;
    wlr.side[1].w4 = -joint_motor[3].velocity;
    wlr.side[1].t1 = -joint_motor[2].torque;
    wlr.side[1].t4 = -joint_motor[3].torque;
    wlr.side[1].wy =  driver_motor[1].velocity;//可能需要加滤波
}

static void chassis_data_output(void)
{
    if (mode.ctrl_mode == PROTECT_MODE) {//保护模式
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        for (int i = 0; i < 4; i++) {
            ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    } else if (mode.chassis_flag.ctrl == CHASSIS_MODE_FORCE) {//力控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1], wlr.side[1].Tw);
        ht_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0.03, wlr.side[0].T4);
        ht_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0.03, wlr.side[0].T1);
        ht_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0.03, -wlr.side[1].T1);
        ht_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0.03, -wlr.side[1].T4);
    } else if (mode.chassis_flag.ctrl == CHASSIS_MODE_POSITION) {//位控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1], wlr.side[1].Tw);
        ht_motor_set_control_para(&joint_motor[0],  wlr.side[0].P4 + joint_motor[0].zero_point,      0, 10, 2, 0);
        ht_motor_set_control_para(&joint_motor[1],  wlr.side[0].P1 - joint_motor[1].zero_point - PI, 0, 10, 2, 0);
        ht_motor_set_control_para(&joint_motor[2], -wlr.side[1].P1 + joint_motor[2].zero_point + PI, 0, 10, 2, 0);
        ht_motor_set_control_para(&joint_motor[3], -wlr.side[1].P4 - joint_motor[3].zero_point,      0, 10, 2, 0);
    } else {//错误时
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        for (int i = 0; i < 4; i++) {
            ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    }
}

void chassis_task(void const *argu)
{
    chassis_init();
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        if (mode.chassis_flag.reset == 1)
            joint_motor_reset();
        chassis_data_input();
        wlr_control();
        chassis_data_output();
        osDelayUntil(&thread_wake_time, 2);
    }
}
