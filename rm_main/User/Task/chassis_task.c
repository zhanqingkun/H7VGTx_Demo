#include "chassis_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "wlr.h"
#include "can_comm.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"
#include "prot_imu.h"
#include "prot_dr16.h"
#include "pid.h"
#include "math_lib.h"
#include "arm_math.h"
#include "string.h"
#include "cmsis_os.h"

#define JOINT_MOTOR_RESET_TORQUE 1.5f
#define JOINT_MOTOR_RESET_ERROR 0.005f

chassis_t chassis;
chassis_scale_t chassis_scale = {
    .remote = 1.0f/660*3,
    .keyboard = 3
};

static void joint_motor_reset(void)
{
    static motor_reset_t motor_reset[4];
    uint32_t thread_wake_time = osKernelSysTick();
    //清除上次复位数据和电机数据 重新初始化
    dji_motor_set_torque(&driver_motor[0], 0);
    dji_motor_set_torque(&driver_motor[1], 0);
    chassis.joint_motor_reset = 0;
    for (int i = 0; i < 4; i++) {
        motor_reset[i].reset_flag = 0;
//        ht_motor_set_control_cmd(&joint_motor[i], CMD_MOTOR_MODE);
//        osDelayUntil(&thread_wake_time, 1);
        ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
//        osDelayUntil(&thread_wake_time, 1);
    }
    //未解锁 等待
    while (!lock_flag)
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
    reset_flag = 0;
}

static void chassis_init()
{
    memset(&chassis, 0, sizeof(chassis_t));
    wlr_init();
	joint_motor_reset();
    wlr.yaw_set = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
}

static void chassis_mode_switch(void)
{
    /* 系统历史状态 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    /* 按键扫描 */
    key_scan(KEY_CHASSIS_ROTATE);
    key_scan(KEY_CHASSIS_POWER);
    key_scan(KEY_CHASSIS_HEIGHT);
    key_scan(KEY_CHASSIS_FIGHT);
    key_scan(KEY_CHASSIS_UNFOLLOW);
    /* 底盘状态切换 */
    switch (ctrl_mode) {
    case PROTECT_MODE: { //能量模式和保护模式下，底盘行为相同
        chassis.mode = CHASSIS_MODE_PROTECT;
        break;
    }
    case REMOTER_MODE: {
        if (last_ctrl_mode != REMOTER_MODE) { //切入遥控模式，初始化底盘模式
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
        }
        /* 底盘小陀螺模式 */
        static uint8_t spin_flag; //单次触发使能标志
        if (ABS(rc.ch5) < 10) { //使能底盘模式切换
            spin_flag = 1;
        }
        if (rc.ch5 == 660 && spin_flag) {
            spin_flag = 0;
            if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE) {
                chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
            } else if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW) {
                chassis.mode = CHASSIS_MODE_REMOTER_ROTATE;
            }
        }
        /* 遥控器注释底盘 */
        if (rc_fsm_check(RC_RIGHT_RD) || rc_fsm_check(RC_LEFT_LD) || rc_fsm_check(RC_LEFT_RD)) { //遥控器注释底盘
            chassis.mode = CHASSIS_MODE_PROTECT;
        }
        break;
    }
    case VISION_MODE: {
//        if (vision.mode == vMODE_bENERGY || vision.mode == vMODE_sENERGY) {
//            kb_status[KEY_CHASSIS_UNFOLLOW] = 1; //视觉能量机关时，切入不跟随状态！！！！！！！！！！！！！！！
//            chassis.mode = CHASSIS_MODE_KEYBOARD_UNFOLLOW; //进入底盘保护模式（击打能量机关）
//        } else 
        if (kb_status[KEY_CHASSIS_ROTATE]) { //进入小陀螺模式
            chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
        } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
            chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
        } else if (kb_status[KEY_CHASSIS_UNFOLLOW]) {
            chassis.mode = CHASSIS_MODE_KEYBOARD_UNFOLLOW;
        } else { //正常模式
            chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
        }
        break;
    }
    case KEYBOARD_MODE: { //键盘模式下：(跟随，陀螺，迎敌三种模式相互切换),(跟随与补给模式相互切换)
        /* 底盘模式切换 */
        switch (chassis.mode) {
        case CHASSIS_MODE_KEYBOARD_FOLLOW: { //键盘跟随模式下
            if(kb_status[KEY_SHOOT_HOUSE] && !wlr.high_flag) { //趴倒模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_PRONE;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入键盘陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            } else if (kb_status[KEY_CHASSIS_UNFOLLOW]) { //进入不跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_UNFOLLOW;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_ROTATE: { //键盘陀螺模式下
            if (!kb_status[KEY_CHASSIS_ROTATE]) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FIGHT: { //键盘迎敌模式下
            if (!kb_status[KEY_CHASSIS_FIGHT]) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入小陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_UNFOLLOW]) { //进入不跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_UNFOLLOW;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW: { //键盘不跟随模式下
            if (!kb_status[KEY_CHASSIS_UNFOLLOW]) { //恢复跟随模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
                kb_status[KEY_CHASSIS_FIGHT]=1;
            } else if (kb_status[KEY_CHASSIS_ROTATE]) { //进入小陀螺模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            } else if (kb_status[KEY_CHASSIS_FIGHT]) { //进入迎敌模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
            }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            if(!kb_status[KEY_SHOOT_HOUSE]) { //趴倒模式
                chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            }
            break;
        }
        default: {
            chassis.mode = CHASSIS_MODE_KEYBOARD_FOLLOW;
            break;
        }
        }
        break;
    }
    default: break;
    }
    /* 系统历史状态更新 */
    last_ctrl_mode = ctrl_mode;
    /* 状态标志复位 */
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_ROTATE)
        key_status_clear(KEY_CHASSIS_ROTATE);
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_FIGHT)
        key_status_clear(KEY_CHASSIS_FIGHT);
    if (chassis.mode != CHASSIS_MODE_KEYBOARD_UNFOLLOW)
        key_status_clear(KEY_CHASSIS_UNFOLLOW);
    if(chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE)
        key_status_clear(KEY_SHOOT_HOUSE);
}

static void chassis_data_input(void)
{
    static chassis_mode_e last_chassis_mode = CHASSIS_MODE_PROTECT;
    //速度输入 模式输入
    switch (chassis.mode) {
        case CHASSIS_MODE_PROTECT: {  //底盘保护模式
            chassis.input.vx = 0;
            chassis.input.vy = 0;
            wlr.ctrl_mode = 0;
            key_status_clear(KEY_CHASSIS_HEIGHT);
            break;
        }
        case CHASSIS_MODE_REMOTER_FOLLOW:  //底盘遥控跟随模式 底盘遥控陀螺模式
        case CHASSIS_MODE_REMOTER_ROTATE: {
            chassis.input.vx = rc.ch4 * chassis_scale.remote; //目标速度
            chassis.input.vy = rc.ch3 * chassis_scale.remote;
            if(rc.sw2 == RC_MI || rc.sw2 == RC_DN)
                wlr.ctrl_mode = 2; //轮腿模式
            else
                wlr.ctrl_mode = 1; //锁腿开轮
           //高度模式
           if (wlr.ctrl_mode == 2) {  //轮腿模式下才可控制腿长
               if(rc.sw2 == RC_MI)
                   wlr.high_flag = 0;
               else if(rc.sw2 == RC_DN)
                   wlr.high_flag = 1;
           } else {
               wlr.high_flag = 0;
           }
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FOLLOW:  //跟随模式可以跳跃  底盘键盘跟随/陀螺/迎敌模式
            if (rc.kb.bit.CTRL && wlr.jump_flag == 0)
                wlr.jump_flag = 1;
            else if (!rc.kb.bit.CTRL)
                wlr.jump_flag = 0;
        case CHASSIS_MODE_KEYBOARD_ROTATE:
        case CHASSIS_MODE_KEYBOARD_FIGHT:
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW:
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            //高速模式
            if (kb_status[KEY_CHASSIS_POWER] == KEY_RUN) {
                chassis_scale.keyboard = 4.0f;  //待修改 高速模式下
                wlr.shift_flag = 1;
            } else {
                chassis_scale.keyboard = 3.0f;  //待修改 普通模式下
                wlr.shift_flag = 0;
            }
            //速度输入
            if (rc.kb.bit.W) {
                chassis.input.vx = chassis_scale.keyboard;
            } else if (rc.kb.bit.S) {
                chassis.input.vx = -chassis_scale.keyboard;
            } else {
                chassis.input.vx = 0;
            }
            if (rc.kb.bit.D) {
                chassis.input.vy = chassis_scale.keyboard;
            } else if (rc.kb.bit.A) {
                chassis.input.vy = -chassis_scale.keyboard;
            } else {
                chassis.input.vy = 0;
            }
            if (chassis.input.vx != 0 && chassis.input.vy != 0) {
                chassis.input.vx /= 1.414f;
                chassis.input.vy /= 1.414f;
            }
            //控制模式
            wlr.ctrl_mode = 2; //键盘直接是轮腿模式
            if (chassis.mode == CHASSIS_MODE_KEYBOARD_PRONE) {
                wlr.ctrl_mode = 1; //趴倒是锁腿开轮
            }            
            //高度模式
            if(kb_status[KEY_CHASSIS_HEIGHT] == KEY_RUN && 
                chassis.mode != CHASSIS_MODE_KEYBOARD_PRONE &&
                chassis.mode != CHASSIS_MODE_KEYBOARD_ROTATE) {
                wlr.high_flag = 1;  //趴倒模式不能起
            } else {
                wlr.high_flag = 0;
                key_status_clear(KEY_CHASSIS_HEIGHT);
            }
            //刹车模式
            if(rc.kb.bit.Q) {
                wlr.stop_flag = 1;
                chassis.input.vx = 0;
                chassis.input.vy = 0;
            } else {
                wlr.stop_flag = 0;
            }
            break;
        }
        default:break;
    }
    //旋转数据输入
    switch (chassis.mode) {
        case CHASSIS_MODE_PROTECT: {
            wlr.yaw_set = (float)yaw_motor.ecd / 8192 * 2 * PI;
            wlr.yaw_fdb =  (float)yaw_motor.ecd / 8192 * 2 * PI;
            break;
        }
        case CHASSIS_MODE_REMOTER_FOLLOW:
        case CHASSIS_MODE_KEYBOARD_FOLLOW:
        case CHASSIS_MODE_KEYBOARD_PRONE: {
            wlr.yaw_set = (float)CHASSIS_YAW_OFFSET / 8192 * 2 * PI;
            wlr.yaw_fdb =  (float)yaw_motor.ecd / 8192 * 2 * PI;
            break;
        }
        case CHASSIS_MODE_KEYBOARD_FIGHT: {
            wlr.yaw_set = (float)CHASSIS_YAW_FIGHT / 8192 * 2 * PI;
            wlr.yaw_fdb =  (float)yaw_motor.ecd / 8192 * 2 * PI;
            break;
        }
        case CHASSIS_MODE_REMOTER_ROTATE:
        case CHASSIS_MODE_KEYBOARD_ROTATE: {
            wlr.yaw_set += (float)CHASSIS_ROTATE_SPEED * 0.002f;
            wlr.yaw_fdb =  (float)yaw_motor.ecd / 8192 * 2 * PI;
            break;
        }
        case CHASSIS_MODE_KEYBOARD_UNFOLLOW: {
            if (last_chassis_mode != CHASSIS_MODE_KEYBOARD_UNFOLLOW)
                wlr.yaw_set = chassis_imu.yaw;
            wlr.yaw_fdb =  chassis_imu.yaw;
            break;
        }
        default:break;
    }
    last_chassis_mode = chassis.mode;
    //速度坐标系换算 
    if (wlr.yaw_set < 0)
        wlr.yaw_set += 2 * PI;
    else if (wlr.yaw_set > 2 * PI)
        wlr.yaw_set -= 2 * PI;
    wlr.yaw_err = circle_error(wlr.yaw_set, wlr.yaw_fdb, 2 * PI);
    chassis.output.vx = chassis.input.vx;
    chassis.output.vy = chassis.input.vy;
//    chassis.output.vx = chassis.input.vx * arm_cos_f32(wlr.yaw_err) - chassis.input.vy * arm_sin_f32(wlr.yaw_err);
//    chassis.output.vy = chassis.input.vx * arm_sin_f32(wlr.yaw_err) + chassis.input.vy * arm_cos_f32(wlr.yaw_err);

	wlr.v_set = chassis.output.vx;
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
    if (wlr.ctrl_mode == 0) {//保护模式
        wlr_protest();
        dji_motor_set_torque(&driver_motor[0], 0);
        dji_motor_set_torque(&driver_motor[1], 0);
        for (int i = 0; i < 4; i++) {
            ht_motor_set_control_para(&joint_motor[i], 0, 0, 0, 0, 0);
        }
    } else if (wlr.ctrl_mode == 2) {//力控
        dji_motor_set_torque(&driver_motor[0], -wlr.side[0].Tw);
        dji_motor_set_torque(&driver_motor[1], wlr.side[1].Tw);
        ht_motor_set_control_para(&joint_motor[0], 0, 0, 0, 0.03, wlr.side[0].T4);
        ht_motor_set_control_para(&joint_motor[1], 0, 0, 0, 0.03, wlr.side[0].T1);
        ht_motor_set_control_para(&joint_motor[2], 0, 0, 0, 0.03, -wlr.side[1].T1);
        ht_motor_set_control_para(&joint_motor[3], 0, 0, 0, 0.03, -wlr.side[1].T4);
    } else if (wlr.ctrl_mode == 1) {//位控
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
    uint32_t thread_wake_time = osKernelSysTick();
    chassis_init();
    for(;;)
    {
        if (reset_flag == 1)
            joint_motor_reset();
        thread_wake_time = osKernelSysTick();
//        taskENTER_CRITICAL();
        chassis_mode_switch();
        chassis_data_input();
        wlr_control();
        chassis_data_output();
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
