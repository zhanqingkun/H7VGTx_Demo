#ifndef __MODE_SWITCH_TASK_H
#define __MODE_SWITCH_TASK_H

#include "stdint.h"

typedef enum
{
    PROTECT_MODE = 0,
    REMOTER_MODE,
    KEYBOARD_MODE
} ctrl_mode_e;

typedef enum
{
    GIMBAL_MODE_PROTECT = 0,
    GIMBAL_MODE_NORMAL,
    GIMBAL_MODE_VISION
} gimbal_mode_e;

typedef enum
{
	CHASSIS_MODE_PROTECT = 0,
	CHASSIS_MODE_FOLLOW,
	CHASSIS_MODE_ROTATE,
	CHASSIS_MODE_FIGHT,
	CHASSIS_MODE_UNFLLOW,
	CHASSIS_MODE_PRONE
} chassis_mode_e;

typedef enum
{
	CHASSIS_MODE_POSITION = 0,
	CHASSIS_MODE_FORCE,
} chassis_ctrl_e;

typedef enum
{
    SHOOT_MODE_PROTECT = 0,
    SHOOT_MODE_CLOSE,
    SHOOT_MODE_OPEN
} shoot_mode_e;

typedef enum
{
    SUPERCAP_MODE_CLOSE = 0,
    SUPERCAP_MODE_OPEN
} cap_mode_e;

typedef struct
{
    ctrl_mode_e ctrl_mode;
    gimbal_mode_e gimbal_mode;
    chassis_mode_e chassis_mode;
    __packed struct
    {
        chassis_ctrl_e ctrl :1;
        uint16_t hight:1;
        uint16_t stop :1;
        uint16_t shift:1;
        uint16_t jump :1;
        uint16_t reset:1;
    } chassis_flag;
    shoot_mode_e shoot_mode;
    cap_mode_e cap_mode;
    uint8_t lock_flag;
} mode_t;

extern mode_t mode;

void mode_switch_task(void const *argu);

#endif
