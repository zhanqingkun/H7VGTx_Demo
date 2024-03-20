#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"

typedef struct
{
    uint8_t stop_cnt;
    uint8_t reset_flag;
    float last_position;
} motor_reset_t;

typedef struct
{
    uint8_t joint_motor_reset;
} chassis_t;

extern chassis_t chassis;

void chassis_task(void const *argu);

#endif
