#ifndef __MATH_CALCU_H
#define __MATH_CALCU_H

#include "math.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define PI 3.14159265358979323846f
#define E  2.718282f
#define SIGN(x) ((x)>0?(1):((x)<0?(-1) 0))
#define ABS(x) ((x>0)?(x):(-(x)))
#define ROUND(type, x) ((type)((x)>0?(x)+0.5f:(x)-0.5f))
#define OUTPUT_LIMIT(output,max,min) \
	((output)<=(max)&&(output)>=(min) ? output : ((output)>(max) ? (output=max):(output=min)))

#define WLR_ADD(x, inc, max) do{x += inc; if (x>=max) x = max;}while(0)
#define WLR_SUB(x, dec, min) do{x -= dec; if (x<=min) x = min;}while(0)
#define WLR_RTN(x, tar, inc)					\
	do{											\
		if(x > tar)								\
		{										\
			x -= inc;							\
			if(x < tar)							\
				x = tar;						\
		}										\
		else if(x < tar)						\
		{										\
			x += inc;							\
			if(x > tar)							\
				x = tar;						\
		}										\
	} while (0)

typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

typedef struct
{
    float max;
    float min;
    float now;
    float last;
    int8_t out;  //发生跳变的方向
} delay_loop_t;

void Data_Limit(float *data, float max, float min);
void ABS_Limit(float *a, float abs_max, float offset);
float Sigmoid_Function(float x);
void Bubble_Sort(float *a, uint8_t n);
float Circle_Error(float *set , float *get , float circle_para);
float Ramp_Input(float fdb, float ref, float slope);
void Ramp_Calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
void Least_Square_Linear_Fit(float x[], float y[], const int num, float* a, float* b);
float Vector_Arg(float x, float y, float x_th, float y_th);
int8_t Delay_Loop(delay_loop_t* dlp, float now_data, float max, float min);

#endif
