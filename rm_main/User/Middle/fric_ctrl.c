#include "fric_ctrl.h"
#include "control_def.h"
#include "tim.h"

#define Init_PWM 900
#define FRIC_CTRL_PERIOD 2

fric_t fric;

uint8_t fric_init(void)
{
    if (!fric.init_flag) {
        fric.mode = FIRC_MODE_STOP;
        //900-2000
        //启动时，油门打到最低
        HAL_TIM_PWM_Start(FricMotor_Time_CH1);
        HAL_TIM_PWM_Start(FricMotor_Time_CH2);

        FricMotor_PWM1 = Init_PWM;
        FricMotor_PWM2 = Init_PWM;
        
        fric.low_pwm = LOW_SPEED;
        fric.mid_pwm = MID_SPEED;
        fric.high_pwm = HIGH_SPEED;
        
        if ((fric.init_cnt++)*FRIC_CTRL_PERIOD > 9000) {
            fric.init_flag = 1;
        } else {
            fric.init_flag = 0;
        }
    }
    return fric.init_flag;
}

static void fric_gun_control(uint16_t pwm)
{
    if (fric.init_flag) {
        FricMotor_PWM1 = Init_PWM+pwm;
        FricMotor_PWM2 = Init_PWM+pwm;
    }
}

void fric_control(void)
{
    switch (fric.mode) {
        case FIRC_MODE_STOP: fric_gun_control(0);         break;
        case FIRC_MODE_RUN:  fric_gun_control(fric.high_pwm);  break;
        default: break;
    }
}
   
