#include "mode_switch_task.h"
#include "prot_dr16.h"
#include "cmsis_os.h"

mode_t mode;

static void remote_unlock(void)
{
    if (rc.sw1 == RC_MI && rc.sw2 == RC_UP) {
        if (rc.ch3 == 660 && rc.ch4 == -660) {
            mode.lock_flag = 1;
        }
    }
}

static void remote_reset(void)
{
    static uint8_t reset_press = 0;
    if (rc.sw1 == RC_MI && rc.sw2 == RC_UP && rc.ch1 == -660 && rc.ch2 == -660 && reset_press == 0) {
        mode.chassis_flag.reset = 1;
        reset_press = 1;
    } else if (!(rc.sw1 == RC_MI && rc.sw2 == RC_UP && rc.ch1 == -660 && rc.ch2 == -660)) {
        reset_press = 0;
         mode.chassis_flag.reset = 0;
    }
}

static void get_keyboard_mode(void)
{
    key_scan(KB_Q);
    key_scan(KB_E);
    key_scan(KB_R);
    key_scan(KB_F);
    key_scan(KB_G);
    key_scan(KB_Z);
    key_scan(KB_X);
    key_scan(KB_C);
    key_scan(KB_V);
    key_scan(KB_B);
    key_scan(KB_CTRL);
}

static void get_sw_mode(void)
{
    get_keyboard_mode();
    if (rc.sw1 == RC_MI && rc.sw2 == RC_UP)
        mode.ctrl_mode = PROTECT_MODE;
    else if (rc.sw1 == RC_DN)
        mode.ctrl_mode = KEYBOARD_MODE;
    else
        mode.ctrl_mode = REMOTER_MODE;

    if (mode.ctrl_mode == PROTECT_MODE) {
        mode.chassis_mode = CHASSIS_MODE_PROTECT;
    } else {
        mode.chassis_mode = CHASSIS_MODE_FOLLOW;
        if (rc.sw2 == RC_UP) {
            mode.chassis_flag.ctrl = CHASSIS_MODE_POSITION;
            mode.chassis_flag.hight = 0;
            mode.chassis_flag.stop = 0;
        } else if (rc.sw2 == RC_MI) {
            mode.chassis_flag.ctrl = CHASSIS_MODE_FORCE;
            mode.chassis_flag.hight = 0;
            mode.chassis_flag.stop = 0;
        } else if (rc.sw2 == RC_DN) {
            mode.chassis_flag.ctrl = CHASSIS_MODE_FORCE;
            mode.chassis_flag.hight = 1;
            mode.chassis_flag.stop = 0;
        }
    }
}

void mode_switch_task(void const *argu)
{
    for(;;)
    {
        if (!mode.lock_flag) {
            remote_unlock();
        } else {
            remote_reset();
            get_sw_mode();
        }
        osDelay(10);
    }
}
