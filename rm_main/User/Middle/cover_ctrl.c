#include "cover_ctrl.h"
#include "control_def.h"
#include "tim.h"

house_mode_e house_mode;

void house_init(void)
{
    //开始系统为保护模式，不使能pwm，无力
    Magazine_PWM = COVER_PWM_CLOSE;
}

void house_control(void)
{
    switch (house_mode) {
        case HOUSE_MODE_OPEN:  {
            HAL_TIM_PWM_Start(Magazine_Time_CH); //使能pwm
            Magazine_PWM = COVER_PWM_OPEN;
            break;
        }
        case HOUSE_MODE_CLOSE:  {
            HAL_TIM_PWM_Start(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_CLOSE;
            break;
        }
        case HOUSE_MODE_PROTECT: {
            HAL_TIM_PWM_Stop(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_CLOSE;
            break;
        }
        default:break;
    }
}
