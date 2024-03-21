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
    gimbal_pid_t yaw_angle, yaw_spd;
    gimbal_pid_t yaw_mecd, yaw_mspd;
    gimbal_pid_t pit_angle, pit_spd;
    gimbal_pid_t pit_mecd, pit_mspd;
} gimbal_t;

extern gimbal_t gimbal;

void gimbal_task(void const *argu);

#endif
