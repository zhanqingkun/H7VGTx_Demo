#include "us_time.h"

prv_us_time_t prv_us_time;

//得到当前计数值
static US_TIME_TYPE usTime_Get(void)
{
	return US_TIME_HTIM.Instance->CNT;
}

//两次间隔时间
static void usTime_Minus(us_time_t* us_time)
{
	if(us_time->now_cnt == us_time->last_cnt)		//没有溢出
		us_time->dt = (us_time->now_time - us_time->last_time) * 1.0e-3 * US_TIME_PRECISION;
	else if (us_time->now_cnt > us_time->last_cnt)	//发生溢出
		us_time->dt = 1.0e-3 * US_TIME_PRECISION * 
		((us_time->now_cnt - us_time->last_cnt ) * (US_TIME_PERIOD + 1) + (us_time->now_time - us_time->last_time));
}

//启动定时器及其溢出中断
void usTime_Start(void)
{
	US_TIME_HTIM.Instance->ARR = (US_TIME_TYPE)US_TIME_PERIOD;
	HAL_TIM_Base_Start_IT(&US_TIME_HTIM);
}

//关闭定时器及其溢出中断
void usTime_End(void)
{
	HAL_TIM_Base_Stop_IT(&US_TIME_HTIM);
}

//周期测试时间
float usTime_Period_Test(us_time_t* us_time)
{
	us_time->now_time = usTime_Get();
	us_time->now_cnt = prv_us_time.overflow_cnt;
	usTime_Minus(us_time);
	us_time->last_time = us_time->now_time;
	us_time->last_cnt = us_time->now_cnt;
	return us_time->dt;
}

//区间测试时间 开始
void usTime_Interval_Test_Start(us_time_t* us_time)
{
	us_time->last_time = usTime_Get();
	us_time->last_cnt = prv_us_time.overflow_cnt;
	us_time->interval_start_flag = 1;
}

//区间测试时间 结束
float usTime_Interval_Test_End(us_time_t* us_time)
{
	if(us_time->interval_start_flag)
	{
		us_time->now_time = usTime_Get();
		us_time->now_cnt = prv_us_time.overflow_cnt;
		usTime_Minus(us_time);
		us_time->interval_start_flag = 0;
	}
	else
		us_time->dt = -1;
	return us_time->dt;
}

//us延时函数
void usTime_Delay(US_TIME_TYPE us)
{
	//第一次溢出需要当前计数值
	prv_us_time.predict_overflow_cnt = prv_us_time.overflow_cnt;
	if(us > (US_TIME_TYPE)US_TIME_PERIOD - usTime_Get())
	{
		us -= ((US_TIME_TYPE)US_TIME_PERIOD - usTime_Get() + 1);
		prv_us_time.predict_overflow_cnt++;
		prv_us_time.predict_time = us;
	}
	else
		prv_us_time.predict_time = us + usTime_Get();
	//多次溢出计算次数和最后计数值
	while(us > (US_TIME_TYPE)US_TIME_PERIOD)
	{
		us -= ((US_TIME_TYPE)US_TIME_PERIOD + 1);
		prv_us_time.predict_overflow_cnt++;
		prv_us_time.predict_time = us;
	}
	//当溢出计数和计数值达到要求时结束
	while(!(usTime_Get() >= prv_us_time.predict_time && prv_us_time.overflow_cnt >= prv_us_time.predict_overflow_cnt));
}
