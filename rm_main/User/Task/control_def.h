#ifndef __CONTROL_DEF_H
#define __CONTROL_DEF_H

#include "stdint.h"

/*----------------------------- player preference ----------------------------- */
#define KEY_CHASSIS_FIGHT       KB_F
#define KEY_CHASSIS_ROTATE      KB_SHIFT
#define KEY_CHASSIS_POWER       KB_C
#define KEY_CHASSIS_UNFOLLOW    KB_G
#define KEY_CHASSIS_STOP        KB_NULL
#define KEY_CHASSIS_HEIGHT      KB_NULL
#define KEY_CHASSIS_JUMP        KB_NULL

#define KEY_GIMBAL_TURN_R       KB_E //KB_E
#define KEY_GIMBAL_TURN_L       KB_Q //KB_Q

#define KEY_SHOOT_HOUSE         KB_B

/*-----------------------------shoot-----------------------------*/
#define LOW_SPEED           0       //526
#define MID_SPEED	        450
#define HIGH_SPEED	        450    //535

//弹舱盖
#define COVER_PWM_OPEN   600
#define COVER_PWM_CLOSE  2400
#define Magazine_PWM     TIM3->CCR2
#define Magazine_Time_CH (&htim3),TIM_CHANNEL_2

//枪管pwm
#define FricMotor_PWM1      TIM3->CCR1
#define FricMotor_PWM2      TIM3->CCR1
#define FricMotor_Time_CH1  (&htim3),TIM_CHANNEL_1
#define FricMotor_Time_CH2  (&htim3),TIM_CHANNEL_1

//拨盘频率
#define TRIGGER_PERIOD      90//ms

/*-----------------------------chassis---------------------------*/
#define SPEED_40W  	3200.0f
#define SPEED_45W   3400.0f
#define SPEED_50W		5000.0f
#define SPEED_55W		5300.0f
#define SPEED_60W		5600.0f
#define SPEED_80W		6200.0f
#define SPEED_100W      7000.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8800.0f
#define SPEED_0W        SPEED_50W  //裁判系统底盘功率调为规则外时
#define SPEED_SUPPLY    2000.0f

#define SUPERCAP_CHAGER_VOLAGE    23.6f
#define SUPERCAP_DISCHAGER_VOLAGE	13.5f //超级电容放电电压下限

#define CHASSIS_YAW_OFFSET  6140
#define CHASSIS_YAW_FIGHT   ((CHASSIS_YAW_OFFSET - 8192/4) % 8192)
#define CHASSIS_ROTATE_SPEED 9 //rad/s

/*-----------------------------gimbal----------------------------*/

#define GIMBAL_PIT_CENTER_OFFSET    5400
#define GIMBAL_PIT_MAX              6400
#define GIMBAL_PIT_MIN              5000

#define GIMBAL_YAW_CENTER_OFFSET    3450
#define GIMBAL_YAW_BETWEEN_ECD      ( 8191 / 4 )
#define FIGHT_OFFSET_ERR            ( -1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI )

#endif
