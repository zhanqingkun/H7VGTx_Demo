#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp, ki, kd;
    float i_max, out_max;
    float ref, fdb;
    float err[2];
    float p_out, i_out, d_out, output;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float i_max, float out_max);
float pid_calc(pid_t *pid, float ref, float fdb);

#endif
