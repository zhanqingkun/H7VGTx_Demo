#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "pid.h"
#include "stdint.h"

#define SHOOT_PERIOD 2

typedef struct
{
    pid_t pid;
    float ref, fdb;
} shoot_pid_t;

typedef enum
{
    TRIGGER_MODE_PROTECT,
    TRIGGER_MODE_STOP,
    TRIGGER_MODE_SINGLE,
    TRIGGER_MODE_SERIES
} shoot_trigger_mode_e;

typedef struct
{
    float fdb;//射速反馈置（m/s）
    float std;//射速标准差
    float mid;
    float max;
    float min;
} shoot_speed_test_t;

typedef struct
{
    uint32_t    shoot_cnt;      //发射子弹数
    float       heat;           //当前热量
    int32_t     heat_remain;    //剩余热量
    uint16_t    heat_max;       //最大热量	裁判系统读出
    uint16_t    cooling_rate;	//冷却速率  裁判系统读出
    uint32_t    shoot_period;   //射击周期
    shoot_speed_test_t bullet_spd;
} barrel_param_t;

typedef struct
{
    shoot_trigger_mode_e trigger_mode;
    shoot_pid_t          trigger_ecd, trigger_spd;
    float                trigger_output;        //发射器力矩
    barrel_param_t       barrel;
    uint16_t             shoot_speed_ref;    //射速设定值（m/s）
    float                trigger_period;	    //拨盘拨出一颗子弹的周期，体现射频
    uint8_t              shoot_speed_vision; //发给视觉的射速档位
} shoot_t;

extern shoot_t shoot;

void shoot_task(void const *argu);

#endif
