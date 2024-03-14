#include "math_lib.h"

//数据限幅函数
void Data_Limit(float *data, float max, float min)
{
	if(*data >= max)
		*data = max;
	else if(*data <= min)
		*data = min;
}
void ABS_Limit(float *a, float abs_max,float offset)
{
	if(*a > abs_max + offset)
		*a = abs_max + offset;
	if(*a < -abs_max + offset)
		*a = -abs_max + offset;
}

//sigmoid函数:将(负无穷,正无穷)映射为(0,1),其中0映射为0.5
float Sigmoid_Function(float x)
{
	float y;
	y= 1 / (1 + powf(E, -x));
	return y;
}

//冒泡排序函数:从大到小排序
void Bubble_Sort(float *a, uint8_t n)
{
	float buf;
	for(uint8_t i=0; i<n; i++)
	{
		for(uint8_t j=0; j<n-1-i; ++j)
		{
			if(a[j] < a[j+1])
			{
				buf = a[j];
				a[j] = a[j+1];
				a[j+1] = buf;
			}
		}
	}
}

//环形数据计算偏差值
//set 设定值 get采样值 circle_para 一圈数值
//环形数据下，直接计算出PID中的偏差值
float Circle_Error(float *set , float *get , float circle_para)
{
	float error;
	if(*set > *get)
	{
		if(*set - *get > circle_para / 2)
			error = *set - *get - circle_para;
		else
			error = *set - *get;
	}
	else if(*set < *get)
	{
		if(*set - *get < -1 * circle_para / 2)
			error = *set - *get + circle_para;
		else
			error = *set - *get;
	}
	else
		error = 0;
	return error;
}

//斜波函数计算:根据输入的值进行叠加
float Ramp_Input(float fdb, float ref, float slope)
{
	if (ref - fdb > slope)
		fdb += slope;
	else if (ref - fdb < -slope)
		fdb -= slope;
	else
		fdb = ref;
	return fdb;
}
void Ramp_Calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
	ramp_source_type->max_value = max;
	ramp_source_type->min_value = min;
	ramp_source_type->frame_period = frame_period;
	ramp_source_type->input = input;
	
	ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
	
	if(ramp_source_type->out > ramp_source_type->max_value)
		ramp_source_type->out = ramp_source_type->max_value;
	else if(ramp_source_type->out < ramp_source_type->min_value)
		ramp_source_type->out = ramp_source_type->min_value;
}

//最小二乘法直线拟合 y=ax+b，计算系数a，b
//x[] 拟合点横坐标  
//y[] 拟合点纵坐标
//num 拟合点数
void Least_Square_Linear_Fit(float x[], float y[], const int num, float* a, float* b)
{
    float sum_x2 = 0.0f;
    float sum_y  = 0.0f;
    float sum_x  = 0.0f;
    float sum_xy = 0.0f;

    for (int i = 0; i < num; i++)
    {
        sum_x2 += x[i]*x[i];
        sum_y  += y[i];
        sum_x  += x[i];
        sum_xy += x[i]*y[i];
    }

    *a = (num*sum_xy - sum_x*sum_y)/(num*sum_x2 - sum_x*sum_x);
    *b = (sum_x2*sum_y - sum_x*sum_xy)/(num*sum_x2-sum_x*sum_x);
}

//复数辐角计算
//x     复数x坐标
//y     复数y坐标
//x_th  x坐标浮点死区,边界属于死区,x绝对值小于它就视为0
//y_th  y坐标浮点死区,边界属于死区,y绝对值小于它就视为0
//float 复数辐角(-PI,PI] (rad),零向量辐角默认为0
float Vector_Arg(float x, float y, float x_th, float y_th)
{
	float arg = 0;
	x_th = x_th > 0 ? x_th : -x_th;
	y_th = y_th > 0 ? y_th : -y_th;
	
	if(x > x_th)
	{
		if(y > y_th || y < -y_th)
			arg = atanf(y/x);           /* 第一、四象限 */
		else
			arg = 0;                    /* x正半轴 */
	}
	else if(x >= -x_th && x <= x_th)
	{
		if(y > y_th)
			arg = PI / 2;               /* y正半轴 */
		else if (y < -y_th)
			arg = -PI / 2;              /* y负半轴 */
		else
			arg = 0;                    /* 原点死区 */
	}
	else if(x < -x_th)
	{
		if(y > y_th)
			arg = atanf(y/x) + PI;      /* 第二象限 */
		else if (y < -y_th)
			arg = atanf(y/x) - PI;      /* 第三象限 */
		else
			arg = PI;                   /* x负半轴 */
	}
	return arg;
}

//滞环死区控制
int8_t Delay_Loop(delay_loop_t* dlp, float now_data, float max, float min)
{
	dlp->now = now_data;
	dlp->max = max;
	dlp->min = min;
	if (dlp->now > dlp->max && dlp->last < dlp->max)		/* 正跳变 */
	dlp->out = 1;
	else if (dlp->now < dlp->min && dlp->last > dlp->min)	/* 负跳变 */
		dlp->out = -1;
	dlp->last = dlp->now;
	return dlp->out;
}
