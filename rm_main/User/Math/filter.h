#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

//一阶低通滤波器
typedef enum
{
	LPFO_MODE_N = 0,
	LPFO_MODE_D = 1,
} lpfo_mode_e;

typedef struct
{
	uint8_t mode : 1;		//是否使用动态调节
	float K;				//滤波系数

	float last_data;		//上一次获取数据
	float now_data;			//本次获取的数据

	float last_after_data;	//上一次获取数据
	float now_after_data;	//本次获取的数据

	float Kw;				//数据稳定时系数
	uint8_t dir_last;		//数据变化方向
	uint8_t dir_now;
	uint8_t cnt;			//稳定增长（减少）持续时间
	uint8_t Threshold_A;	//阈值1用于一阶带参滤波器，变化角度大于此值时，计数增加
	uint8_t Threshold_T;	//阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
} LPFO_filter_t;

typedef struct
{
		uint8_t mode : 1;
		float K;
		float Kw;
		uint8_t Threshold_A;
		uint8_t Threshold_T;
} LPFO_param_t;

void LPFOfilter_Init(LPFO_filter_t* pft,LPFO_param_t* pftinit);
float LPFOfilter_Calc(LPFO_filter_t* pft, float get);

//平均滤波器
#define AVG_FILTER_MAX_NUM 10

typedef struct
{
	uint8_t filter_num;
	float array[AVG_FILTER_MAX_NUM];
	uint8_t index;
	uint8_t init;
} AVGfilter_t;

float AVGfilter_Calc(AVGfilter_t* avg, float get);

#endif
