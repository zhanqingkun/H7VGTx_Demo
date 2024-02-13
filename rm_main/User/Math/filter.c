#include "filter.h"

//一阶低通滤波器初始化函数 一般模式下，只用调K； 动态模式下，需要调A，T，和Kw
void LPFOfilter_Init(LPFO_filter_t* pft,LPFO_param_t* pftinit)
{
    pft->mode = pftinit->mode;
    pft->K    = pftinit->K;
    pft->last_data = pft->now_data = pft->last_after_data = pft->now_after_data = 0;
    if(pftinit->mode)  //如果启用滤波系数的动态调节
    {
        pft->Kw = pftinit->Kw;
        pft->Threshold_A = pftinit->Threshold_A;
        pft->Threshold_T = pftinit->Threshold_T;
        pft->cnt = 0;
        pft->dir_last = pft->dir_now = 0;
    }
}

//一阶低通滤波器
float LPFOfilter_Calc(LPFO_filter_t* pft, float get)
{
    pft->now_data = get;
    if(pft->mode == LPFO_MODE_D)  //如果使用动态调整
    {
        /*根据连续真实数据判断数据状态*/
        if(pft->now_data > pft->last_data + pft->Threshold_A)  //判断变化方向
            pft->dir_now = 1;
        else if(pft->now_data < pft->last_data - pft->Threshold_A)
            pft->dir_now = 2;
        else
            pft->dir_now = 0;
        pft->last_data = pft->now_data;

        /*增量稳定性判断*/
        if(pft->dir_now == pft->dir_last)		//判断前后两次变化方向是否相等
        {
            pft->cnt+=1;
            if(pft->cnt >= pft->Threshold_T)	//比较 单向变化持续时长
            {
                pft->K += 0.1f;					//提高灵敏度，增强跟随效果
                if(pft->K >= 1) pft->K = 1;
                pft->cnt = pft->Threshold_T;	//防止溢出
            }
        }
        else
        {
            pft->K = pft->Kw;  //数据围绕某值波动时，降低灵敏度，增强滤波效果
        }
        pft->dir_last = pft->dir_now;
    }
    /*一阶低通滤波*/
    pft->now_after_data = pft->K*pft->now_data + (1-pft->K)*pft->last_after_data;
    pft->last_after_data = pft->now_after_data;

    return pft->now_after_data;
}

//平均滤波器:初始化只需设置filter_num且不得大于AVG_FILTER_MAX_NUM
float AVGfilter_Calc(AVGfilter_t* avg, float get)
{
	if(avg->init == 0)
	{
		avg->array[avg->index++] = get;
		if(avg->index==avg->filter_num)
		{
			avg->init = 1;
			avg->index = 0;
		}
		return get;
	}
	else
	{
		float sum = 0;
		if(avg->index==avg->filter_num)
			avg->index = 0;
		avg->array[avg->index++] = get;
		for(uint8_t i = 0; i<avg->filter_num; i++)
			sum += avg->array[i];
		return sum/avg->filter_num;
	}
}
