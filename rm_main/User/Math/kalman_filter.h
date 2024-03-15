//#ifndef __KALMAN_FILTERS_H
//#define __KALMAN_FILTERS_H

//#include "stdint.h"
//#include "arm_math.h"

//#ifndef user_malloc
//    #ifdef _CMSIS_OS_H
//        #define user_malloc pvPortMalloc
//    #else
//        #define user_malloc malloc
//    #endif
//#endif

//#define mat         arm_matrix_instance_f32
//#define mat_init    arm_mat_init_f32
//#define mat_add     arm_mat_add_f32
//#define mat_sub     arm_mat_sub_f32
//#define mat_mult    arm_mat_mult_f32
//#define mat_trans   arm_mat_trans_f32
//#define mat_inv     arm_mat_inverse_f32

////一阶卡尔曼滤波结构体定义,都是标量
//typedef struct
//{
//	float X_last; //上一时刻的最优结果  X(k-|k-1)
//	float X_mid;  //当前时刻的预测结果  X(k|k-1)
//	float X_now;  //当前时刻的最优结果  X(k|k)
//	float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
//	float P_now;  //当前时刻最优结果的协方差  P(k|k)
//	float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
//	float kg;     //kalman增益
//	float A;      //系统参数
//	float B;      //输入项参数
//	float Q;      //状态方程噪声
//	float R;      //观测方程噪声
//	float H;
//} kalman1_param_t;

////二阶卡尔曼五条重要参数中间变量定义，用于计算，创建函数的第一个输入
//typedef struct
//{
//	float raw_value;
//	float filtered_value[2];     //卡尔曼滤波的返回值，0是位置1是速度
//	mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
//} kalman2_filter_t;

////二阶卡尔曼数组初始化，创建函数的第二个输入
//typedef struct
//{
//    float raw_value;
//    float filtered_value[2];        //卡尔曼滤波的返回值，0是位置1是速度
//    float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
//    float P_data[4];
//    float AT_data[4], HT_data[4];
//    float A_data[4];
//    float H_data[4];
//    float Q_data[4];
//    float R_data[4];
//} kalman2_param_t;

////卡尔曼滤波器计算与创建函数
//void Kalman1_Filter_Create(kalman1_param_t *p, float Q, float R);
//void Kalman1_Filter_Reinit(kalman1_param_t *p, float Q, float R);
//void Kalman1_Filter_Deinit(kalman1_param_t *p);
//float Kalman1_Filter_Calc(kalman1_param_t* p, float dat);

//void Kalman2_Filter_Create(kalman2_filter_t *F, kalman2_param_t *I);
//float *Kalman2_Filter_Calc(kalman2_filter_t *F, float signal1, float signal2);
//float Kalman2_Filter_Calc2(kalman2_filter_t *F, float signal1, float signal2);

//#endif
