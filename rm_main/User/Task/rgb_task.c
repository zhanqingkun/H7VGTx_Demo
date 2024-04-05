#include "rgb_task.h"

#include "mode_switch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "chassis_task.h"

#include "trigger_ctrl.h"
#include "fric_ctrl.h"
#include "cover_ctrl.h"

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "drv_ws2812b.h"

void read_status(void)
{
    if (ctrl_mode == REMOTER_MODE || ctrl_mode == KEYBOARD_MODE) {
        rgb_change(1,7);
    } else if (ctrl_mode == VISION_MODE) {
        rgb_change(1,2);
    } else {
        rgb_change(1,0);
    }

	if (chassis.mode == CHASSIS_MODE_REMOTER_FOLLOW || \
        chassis.mode == CHASSIS_MODE_KEYBOARD_FOLLOW) {
        rgb_change(2,7);
    } else if (chassis.mode == CHASSIS_MODE_KEYBOARD_FIGHT) {
        rgb_change(2,2);
    } else if (chassis.mode == CHASSIS_MODE_REMOTER_ROTATE || \
               chassis.mode == CHASSIS_MODE_KEYBOARD_ROTATE) {
        rgb_change(2,1);
    } else if (chassis.mode == CHASSIS_MODE_KEYBOARD_UNFOLLOW) {
        rgb_change(2,3);
    } else {
        rgb_change(2,0);
    }
    
    if (fric.mode == FIRC_MODE_RUN) {
        rgb_change(3,7);
    } else {
        rgb_change(3,0);
    }
    
    if (shoot.trigger_mode == TRIGGER_MODE_SINGLE || shoot.trigger_mode == TRIGGER_MODE_SINGLE) {
        rgb_change(4,1);
    } else {
        rgb_change(4,0);
    }
    
    if (house_mode == HOUSE_MODE_OPEN) {
        rgb_change(5,1);
    } else {
        rgb_change(5,0);
    }
}

void rgb_task(void const *argu)
{
    for(;;)
    {
		read_status();
		rgb_output_data();
        osDelay(100);
    }
}
