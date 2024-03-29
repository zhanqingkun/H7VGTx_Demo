#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "pid.h"
#include "stdint.h"

typedef struct
{
    pid_t pid;
    float ref, fdb;
} gimbal_pid_t;

typedef struct
{
    gimbal_pid_t yaw_angle, yaw_spd, yaw_ecd;
    gimbal_pid_t pit_angle, pit_spd, pit_ecd;
} gimbal_t;

extern gimbal_t gimbal;

void gimbal_task(void const *argu);

#endif
