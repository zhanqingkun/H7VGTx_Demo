#include "func_generator.h"

#define PI 3.14159265358979323846f

//正弦信号发生器初始化
void FGT_sin_Init(FGT_sin_t* sin, float Td, float dc, float T, float A, float phi)
{
	sin->Td = Td;
	sin->time = 0;
	sin->dc = dc;
	sin->A = A;
	sin->T = T;
	sin->phi = phi;
	sin->out = 0;
}

//产生正弦信号
float FGT_sin_Calc(FGT_sin_t* sin)
{
	float ac = 0;
	/*计算交流量*/
	ac = (sin->A) * sinf( (2*PI / sin->T) * (sin->time) + sin->phi);
	/*迭代时间（先输出再计算，使从0开始）*/
	sin->time = fmodf(sin->time+sin->Td, sin->T);
	/*计算交直流量,迭代输出记录*/
	sin->out = ac + sin->dc;
	/*输出角波交直流量*/
	return sin->out;
}

//方波信号发生器初始化
void FGT_sqr_Init(FGT_sqr_t* sqr, float Td, float Th, float Tl, float high, float low)
{
	sqr->Td = Td;
	sqr->time = 0;
	sqr->high = high;
	sqr->low = low;
	sqr->Th = Th;
	sqr->Tl = Tl;
	sqr->out = 0;
}

//产生方波信号
float FGT_sqr_Calc(FGT_sqr_t* sqr)
{
	float dc = 0;
	/*计算周期*/
	float T = sqr->Th + sqr->Tl;
	/*计算直流量*/
	if(sqr->time > sqr->Th)
		dc = sqr->low;
	else
		dc = sqr->high;
	/*迭代时间（先输出再计算，使从0开始）*/
	sqr->time = fmodf(sqr->time+sqr->Td, T);
	/*迭代输出记录*/
	sqr->out = dc;
	/*返回方波交直流量*/
	return sqr->out;
}

//角波信号发生器初始化
void FGT_agl_Init(FGT_agl_t* agl, float Td, float T1, float T2, float T1_out, float T2_out)
{
	agl->Td = Td;
	agl->time = 0;
	agl->T1 = T1;
	agl->T2 = T2;
	agl->T1_out = T1_out;
	agl->T2_out  = T2_out;
	agl->out = 0;
}

//产生角波信号
float FGT_agl_Calc(FGT_agl_t* agl)
{
	float ac  = 0;
	/*计算周期*/
	float T = agl->T1 + agl->T2;
	/*确定区间,计算相对时间，计算交流量*/
	if(agl->time < agl->T1)  //在第一个拐点之前
		ac = ((agl->T1_out-agl->T2_out)/agl->T1) * agl->time + agl->T2_out;
	else if(agl->time >= agl->T1)  //在两个拐点之间
		ac = ((agl->T2_out-agl->T1_out)/agl->T2) * (agl->time - agl->T1) + agl->T1_out;
	/*迭代时间*/
	agl->time = fmodf(agl->time+agl->Td,T);
	/*迭代输出记录*/
	agl->out = ac;
	/*输出角波交直流量*/
	return agl->out;
}

//符号生成器初始化
void FGT_npz_Init(FGT_npz_t* npz, float Td, float T1, float T2, float T3)
{
	npz->Td = Td;
	npz->time = 0;
	npz->T1 = T1;
	npz->T2 = T2;
	npz->T3 = T3;
	npz->out = 0;
}

//产生符号信号
//依次产生T1时间的1，T2时间的0，T3时间的-1信号
float FGT_npz_Calc(FGT_npz_t* npz)
{
	/*计算周期*/
	float T = npz->T1 + npz->T2 + npz->T3;
	/*确定输出*/
	if (npz->time <= npz->T1)  //在第一个拐点之前
		npz->out = 1;
	else if ((npz->T1 < npz->time) && (npz->time < npz->T1 + npz->T2))  //在两个拐点之间
		npz->out = 0;
	else if ((npz->T1 + npz->T2 <= npz->time) && (npz->time <= T))  //在第二个拐点之后
		npz->out = -1;
	/*迭代时间（先输出再计算，使从0开始）*/
	npz->time = fmodf(npz->time+npz->Td,T);
	/*输出符号*/
	return npz->out;
}

//一般函数信号发生器
void FGT_f_Init(FGT_f_t* pf, float (*f)(float time), float Td, float T)
{
	pf->f = f;
	pf->T = T;
	pf->Td = Td;
	pf->time = 0;
	pf->out = 0;
}

//一般函数信号发生器
//主要函数调用周期与结构体的周期，微分量之间的对应关系
float FGT_f_Calc(FGT_f_t* pf)
{
	/*迭代时间（先输出再计算，使从0开始）*/
	pf->time = fmodf(pf->time+pf->Td,pf->T);
	/*计算输出*/
	pf->out = pf->f(pf->time);
	/*返回输出*/
	return pf->out;
}

//读取当前生成的32位无符号随机数
//HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);

//生成[min,max]范围的随机数
int RNG_Get_RandomRange(int min,int max)
{
	uint32_t temp_32_rng;
//	HAL_RNG_GenerateRandomNumber(&hrng,&temp_32_rng);
	return temp_32_rng%(max-min+1)+min;
}

//产生一个高斯噪声点:mu为均值,sigma_f为标准差
float GaussGenerate(float mu, float sigma_f)
{
	float t1,t2,a,r;
	float x;
	//产生两个均匀分布的0~1的随机序列
	t1 = (float)rand()/(RAND_MAX);
	t2 = (float)rand()/(RAND_MAX);
	//极坐标的两个随机变量分布序列
	a = 2*PI*t1;			//a是极坐标的角度：变成了0~2*pi的均匀分布
	r = sqrt(-2*logf(t2));	//r是极坐标的距离：变成自然对数开根号的一种分布
	//用极坐标(a,r)转换成笛卡尔坐标(x,y)，这就是产生的高斯白噪声
	x = r*cos(a);
	return mu+sigma_f*x;
}

//产生一个高斯噪声数组:mu为均值,sigma_f为标准差
void Gauss(float gs[], int lengh, float mu, float sigma_f)
{
	for(int i=0; i<lengh; i++)
		gs[i]=GaussGenerate(mu,sigma_f);
}
