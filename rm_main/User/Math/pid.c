#include "pid.h"

#define LIMIT(x,limit) (x)=(((x)<=(-limit))?(-limit):(((x)>=(limit))?(limit):(x)))

/*
 * @brief     pid初始化
 * @retval    void
 */
void pid_init(pid_t *pid, float kp, float ki, float kd, float i_max, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
}

/*
 * @brief     pid计算
 * @param[in] pid: pid结构体
 * @param[in] ref: 期望值
 * @param[in] fdb: 反馈值
 * @retval    pid计算结果
 */
float pid_calc(pid_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;
    
    pid->p_out = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
    LIMIT(pid->i_out, pid->i_max);
    
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    LIMIT(pid->output, pid->out_max);
    return pid->output;
}
