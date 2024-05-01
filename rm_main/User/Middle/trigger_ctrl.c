#include "trigger_ctrl.h"
#include "mode_switch_task.h"
#include "shoot_task.h"
#include "drv_dji_motor.h"
#include "prot_dr16.h"
//#include "control_def.h"
//#include "math_lib.h"
//#include "pid.h"

#define TRIGGER_MOTOR_ECD   36859.0f  //拨盘一颗子弹转过的编码值 8191 * 36 / 8 = 36859.5f
#define ABS(x) ((x>0)? (x): (-(x)))

float MIN_HEAT = 30;        //热量控制裕量

static uint16_t frequency_cnt = 0;	//射击周期计算
static uint8_t  shoot_enable  = 1;  //单发使能标志
static float trigger_ecd_error;

static void trigger_pid_calc(void)
{
    shoot.trigger_ecd.fdb = trigger_motor.total_ecd;
    shoot.trigger_spd.ref = pid_calc(&shoot.trigger_ecd.pid, shoot.trigger_ecd.ref, shoot.trigger_ecd.fdb);
    shoot.trigger_spd.fdb = trigger_motor.speed_rpm;
    shoot.trigger_output = pid_calc(&shoot.trigger_spd.pid, shoot.trigger_spd.ref, shoot.trigger_spd.fdb);
}

static uint8_t single_shoot_reset(void)
{
    return (
        (rc.mouse.l == 0 && ctrl_mode == KEYBOARD_MODE)
        || (ABS(rc.ch5) < 10 && ctrl_mode == REMOTER_MODE)
    );
}

static uint8_t single_shoot_enable(void)
{
    return (
        shoot_enable
        && shoot.barrel.heat_remain >= MIN_HEAT
        && (rc.mouse.l || rc.ch5 > 500)
        && ABS(trigger_ecd_error) < TRIGGER_MOTOR_ECD
    );
}

static uint8_t series_shoot_enable(void)
{
    return (
        ((ctrl_mode == REMOTER_MODE)// && vision.shoot_enable
            || (ctrl_mode == PROTECT_MODE && (rc.sw2 == RC_MI || rc.sw2 == RC_DN))
            || (ctrl_mode == KEYBOARD_MODE && rc.mouse.l) 
        )
        && shoot.barrel.heat_remain >= MIN_HEAT  //热量控制
        && frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period  //射频控制
        && ABS(trigger_ecd_error) < TRIGGER_MOTOR_ECD  //拨盘误差控制
    );
}

void trigger_init(void)
{
    pid_init(&shoot.trigger_ecd.pid, NONE, 0.3f, 0, 0.3f, 0, 5000);
    pid_init(&shoot.trigger_spd.pid, NONE, 0.0015f, 0, 0, 0, 1);
}

void trigger_control(void)
{
    switch (shoot.trigger_mode) {
        case TRIGGER_MODE_PROTECT: { //拨盘保护模式，保持惯性，无力
            frequency_cnt = 0; //计时变量置0，打出当前一发，禁止
            shoot.barrel.shoot_period = 0;
            shoot.trigger_ecd.ref = trigger_motor.total_ecd;
            shoot.trigger_spd.pid.i_out = 0;
            shoot.trigger_output = 0;
            break;
        }
        case TRIGGER_MODE_STOP: { //拨盘停止模式，保持静止，有力
            frequency_cnt = 0; //计时变量置0，打出当前一发，禁止
            shoot.barrel.shoot_period = 0;
            trigger_pid_calc();
            break;
        }
        case TRIGGER_MODE_SINGLE: { //拨盘单发模式，连续开枪请求，只响应一次
            frequency_cnt++;
            trigger_ecd_error = shoot.trigger_ecd.ref - shoot.trigger_ecd.fdb;
            if (single_shoot_reset()) {
                shoot_enable = 1;
            }
            if (single_shoot_enable()) { //热量控制
                shoot_enable = 0;
                shoot.trigger_ecd.ref += TRIGGER_MOTOR_ECD;
                shoot.barrel.heat += 10;
            }
            trigger_pid_calc();
            break;
        }
        case TRIGGER_MODE_SERIES: { //拨盘连发模式，连续开枪请求，连续响应
            frequency_cnt++;
            trigger_ecd_error = shoot.trigger_ecd.ref - shoot.trigger_ecd.fdb;
            if (series_shoot_enable()) { //一个周期打一颗
                frequency_cnt = 0;
                shoot.trigger_ecd.ref += TRIGGER_MOTOR_ECD;//拨一颗子弹
                shoot.barrel.heat += 10;
            }
            trigger_pid_calc();
            break;
        }
        default: {
            break;
        }
    }
    dji_motor_set_torque(&trigger_motor, shoot.trigger_output);
}
