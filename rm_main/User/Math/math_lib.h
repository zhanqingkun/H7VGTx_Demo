#ifndef __MATH_CALCU_H
#define __MATH_CALCU_H

#define PI 3.14159265358979323846f
#define E  2.718282f
#define SIGN(x) ((x)>0?(1):((x)<0?(-1) 0))
#define ABS(x) ((x>0)?(x):(-(x)))
#define ROUND(type, x) ((type)((x)>0?(x)+0.5f:(x)-0.5f))

typedef struct
{
    float input;
    float out;
    float min, max;
    float frame_period;
} ramp_t;

#endif
