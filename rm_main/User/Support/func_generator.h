#ifndef __FUNC_GENERATOR_H
#define __FUNC_GENERATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "rng.h"

//正弦信号生成器
typedef struct
{
    float Td;   //信号生成周期 即调用函数周期 Td,time,T单位相同(ms或s都行)
    float time; //当前运行时间
    float dc;   //直流量
    float T;    //正弦信号的周期
    float A;    //正弦信号的振幅
    float phi;  //正弦信号的初相
    float out;  //输出记录
} FGT_sin_t;

void FGT_sin_Init(FGT_sin_t* sin, float Td, float dc, float T, float A, float phi);
float FGT_sin_Calc(FGT_sin_t* sin);

//方波信号生成器
typedef struct
{
    float Td;   //信号生成周期 即调用函数周期 Td,time,Th,Tl单位相同(ms或s都行)
    float time; //当前运行时间
    float Th;   //高电平时间
    float Tl;   //低电平时间
    float high; //高电平幅值
    float low;  //低电平幅值
    float out;
} FGT_sqr_t;

void FGT_sqr_Init(FGT_sqr_t* sqr, float Td, float Th, float Tl, float high, float low);
float FGT_sqr_Calc(FGT_sqr_t* sqr);

//角波信号生成器
typedef struct
{
    float Td;     //信号生成周期 即调用函数周期 Td,time,T1,T2单位相同(ms或s都行)
    float time;   //当前运行时间
    float T1;     //周期从0开始，到T1，然后再到T2(也即0)
    float T2;
    float T1_out; //当时间为T1时的输出
    float T2_out; //当时间为T2(也即0)时的输出
    float out;
} FGT_agl_t;

void FGT_agl_Init(FGT_agl_t* agl, float Td, float T1, float T2, float T1_out, float T2_out);
float FGT_agl_Calc(FGT_agl_t* agl);

//符号函数信号生成器
typedef struct
{
    float Td;   //信号生成周期 即调用函数周期 Td,time,T1,T2,T3单位相同(ms或s都行)
    float time; //当前运行时间
    float T1;   //周期从0开始，到T1，然后再到T2，最后到T3(也即0)
    float T2;   //0到T1输出-1，T1到T2输出0，T2到T3输出1
    float T3;
    float out;
} FGT_npz_t;

void FGT_npz_Init(FGT_npz_t* npz, float Td, float T1, float T2, float T3);
float FGT_npz_Calc(FGT_npz_t* npz);

//一般函数信号发生器
typedef struct _FGT_f_t
{
    float Td;   //信号生成周期 即调用函数周期 Td,time,T单位相同(ms或s都行)
    float time; //当前运行时间
    float T;    //函数周期
    float (*f)(float time);
    float out;
} FGT_f_t;

void FGT_f_Init(FGT_f_t* pf, float (*f)(float time), float Td, float T);
float FGT_f_Calc(FGT_f_t* pf);

//生成一维噪声(随机数)
float FGT_RandomGenerate(float min, float max);

//生成高斯噪声
float FGT_GaussGenerate(float mu, float sigma);

#ifdef __cplusplus
}
#endif

#endif
