#include "us_time.h"

prv_us_time_t prv_us_time;

//@breif  得到US_TIME_HTIM当前计数值
//@retval None
static US_TIME_TYPE usTime_Get(void)
{
    return US_TIME_HTIM.Instance->CNT;
}

//@breif  计算两次间隔时间
//@param  us_time: 毫秒定时器结构体
//@retval None
static void usTime_Minus(us_time_t* us_time)
{
    //时间间隔=计数一次时间*(溢出次数差值*溢出一次需要计数次数+计数值差值)
    us_time->dt = US_TIME_PRECISION * ((us_time->now_tim_cnt - us_time->last_tim_cnt) + 
                  (us_time->now_overflow_cnt - us_time->last_overflow_cnt) * (US_TIME_PERIOD + 1));
}

//@breif  启动定时器及其溢出中断
//@retval None
void usTime_Start(void)
{
    US_TIME_HTIM.Instance->ARR = (US_TIME_TYPE)US_TIME_PERIOD;
    HAL_TIM_Base_Start_IT(&US_TIME_HTIM);
}

//@breif  关闭定时器及其溢出中断
//@retval None
void usTime_End(void)
{
    HAL_TIM_Base_Stop_IT(&US_TIME_HTIM);
}

//@breif  周期测试时间
//@param  us_time: 毫秒定时器结构体
//@retval 返回与上次运行此函数所距离的时间
float usTime_Period_Test(us_time_t* us_time)
{
    us_time->now_tim_cnt = usTime_Get();
    us_time->now_overflow_cnt = prv_us_time.overflow_cnt;
    usTime_Minus(us_time);
    us_time->last_tim_cnt = us_time->now_tim_cnt;
    us_time->last_overflow_cnt = us_time->now_overflow_cnt;
    return us_time->dt;
}

//@breif  区间测试时间 开始
//@param  us_time: 毫秒定时器结构体
//@retval None
void usTime_Interval_Test_Start(us_time_t* us_time)
{
    us_time->last_tim_cnt = usTime_Get();
    us_time->last_overflow_cnt = prv_us_time.overflow_cnt;
    us_time->interval_start_flag = 1;
}

//@breif  区间测试时间 结束
//@param  us_time: 毫秒定时器结构体
//@retval 成功返回区间测试时间，失败返回-1
float usTime_Interval_Test_End(us_time_t* us_time)
{
    if(us_time->interval_start_flag)
    {
        us_time->now_tim_cnt = usTime_Get();
        us_time->now_overflow_cnt = prv_us_time.overflow_cnt;
        usTime_Minus(us_time);
        us_time->interval_start_flag = 0;
    }
    else
        us_time->dt = -1;
    return us_time->dt;
}

//@breif  us延时函数
//@param  us: 需要延时的毫秒数
//@retval None
void usTime_Delay(float us)
{
    prv_us_time.tim_cnt = usTime_Get();
    //第一次溢出需要当前计数值
    prv_us_time.predict_overflow_cnt = prv_us_time.overflow_cnt;
    if(us / US_TIME_PRECISION > US_TIME_PERIOD - prv_us_time.tim_cnt)
    {
        us -= ((US_TIME_TYPE)US_TIME_PERIOD - prv_us_time.tim_cnt + 1) * US_TIME_PRECISION;
        prv_us_time.predict_overflow_cnt++;
        prv_us_time.predict_tim_cnt = (US_TIME_TYPE)(us / US_TIME_PRECISION);
    }
    else
        prv_us_time.predict_tim_cnt = (US_TIME_TYPE)(us / US_TIME_PRECISION + prv_us_time.tim_cnt);
    //多次溢出计算次数和最后计数值
    while(us / US_TIME_PRECISION > US_TIME_PERIOD)
    {
        us -= ((US_TIME_TYPE)US_TIME_PERIOD + 1) * US_TIME_PRECISION;
        prv_us_time.predict_overflow_cnt++;
        prv_us_time.predict_tim_cnt = (US_TIME_TYPE)(us / US_TIME_PRECISION);
    }
    //当溢出计数和计数值达到要求时结束
    while(!(prv_us_time.tim_cnt >= prv_us_time.predict_tim_cnt && prv_us_time.overflow_cnt >= prv_us_time.predict_overflow_cnt))
    {
         prv_us_time.tim_cnt = usTime_Get();
    }
}
