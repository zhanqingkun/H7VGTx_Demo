#ifndef __PID_H
#define __PID_H
				
#include "stm32f4xx_hal.h"

typedef struct _pid_struct_t
{
	float kp, ki, kd;
	float i_max, out_max;
	float ref, fdb;
	float err[2];
	float p_out, i_out, d_out, output;
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float i_max, float out_max);           
float PID_Calc(pid_t *pid, float ref, float fdb);

#endif
