//一维、二维卡尔曼滤波器
//1.一维卡尔曼滤波器使用示例：
//使用时先定义一个一维kalman指针，然后调用Kalman1FilterCreate()创建一个滤波器
//每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
//	Kalman1_param_t p;			//定义一个一维卡尔曼滤波器结构体
//	float error_yaw_angle;		//需要进行滤波的数据
//	KalmanCreate(&p,20,200);	//初始化该滤波器的Q=20 R=200参数
//	while(1)
//	{
//		error_yaw_angle = sersor();						//获取数据
//		error_yaw_angle = Kalman1_Filter_Calc(&p, SersorData);	//对数据进行滤波
//	}
//2.二维卡尔曼滤波器使用示例：
//使用时先定义一个结构体指针初始化卡尔曼参数，再定义一个二维kalman结构体指针，用于定义
//计算五大公式所用中间变量，之后调用Kalman2FilterCreate()创建一个滤波器
//	Kalman2_param_t F =
//	{
//		.P_data = {2, 0, 0, 2},
//		.A_data = {1, 0.002, 0, 1},//采样时间间隔
//		.H_data = {1, 0, 0, 1},
//		.Q_data = {1, 0, 0, 1},
//		.R_data = {200, 0, 0, 400}//500 1000
//	};//初始化部分kalman参数
//	    kalman2_filter_t I;			//定义一个二维卡尔曼滤波器结构体
//	    float Xx, vx;				//Xx是位置，vx是速度，作为卡尔曼的两个信号量输入
//	    float *yaw_kf_result ;		//用于存储卡尔曼滤波结果，0是滤波后位置，1是滤波后速度
//	Kalman2_Filter_Create(&F, &I);	//初始化二维卡尔曼滤波器
//	    yaw_kf_result = Kalman2_Filter_Calc(&F,Xx,vx);

#include "kalman_filter.h"

//创建一阶卡尔曼滤波器
//p：卡尔曼结构体；
//Q：状态方程噪声协方差
//R：观测方程协方差
//R越大，传感器方差越大，说明越不信任传感器，认为完全依赖估计；反之认为完全依赖传感器。
void Kalman1_Filter_Create(kalman1_param_t *p, float Q, float R)
{
	memset(p, 0, sizeof(kalman1_param_t));
	p->Q = Q;
	p->R = R;
	p->A = 1;
	p->H = 1;
}

void Kalman1_Filter_Reinit(kalman1_param_t *p, float Q, float R)
{
	p->Q = Q;
	p->R = R;
}

void Kalman1_Filter_Deinit(kalman1_param_t *p)
{ 
	p->X_last	= 0;//上一时刻的最优结果  X(k-|k-1)
	p->X_mid	= 0;//当前时刻的预测结果  X(k|k-1)
	p->X_now	= 0;//当前时刻的最优结果  X(k|k)
	p->P_mid	= 0;//当前时刻预测结果的协方差  P(k|k-1)
	p->P_now	= 0;//当前时刻最优结果的协方差  P(k|k)
	p->P_last	= 0;//上一时刻最优结果的协方差  P(k-1|k-1)
	p->kg		= 0;//kalman增益
}

//一阶卡尔曼滤波器计算
float Kalman1_Filter_Calc(kalman1_param_t* p, float dat)
{
	//-------------------预测-------------------
	//1.对下一时刻均值的预测：X̂k'=AX̂k-1+BUk
	p->X_mid = p->A*p->X_last;                    //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
	//2.对下一时刻方差的预测：Pk'=APk-1A'+Q
	p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q

	//-------------------更新-------------------*/
	//3.卡尔曼增益：Kk=Pk'C'/CPk'C'+R
	p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	//4.状态量更新（加权计算）：X̂k=X̂k'+Kk（Yk-CX̂k'）
	p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	//5.方差更新：Pk=（1-KkC）Pk'
	p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)

	p->P_last = p->P_now;                         //状态更新
	p->X_last = p->X_now;
	return p->X_now;							  //输出预测结果x(k|k)
}

//创建二阶卡尔曼滤波器
//F：卡尔曼结构体,用于计算矩阵及储存滤波结果；
//I：卡尔曼结构体,用于数组初始化；
void Kalman2_Filter_Create(kalman2_filter_t *F,  kalman2_param_t *I)
{
	mat_init(&F->xhat,2,1,(float *)I->xhat_data);//矩阵初始化,中间两个参数为矩阵行列m*n
	mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
	mat_init(&F->z,2,1,(float *)I->z_data);
	mat_init(&F->A,2,2,(float *)I->A_data);
	mat_init(&F->H,2,2,(float *)I->H_data);
	mat_init(&F->Q,2,2,(float *)I->Q_data);
	mat_init(&F->R,2,2,(float *)I->R_data);
	mat_init(&F->P,2,2,(float *)I->P_data);
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
	mat_init(&F->K,2,2,(float *)I->K_data);
	mat_init(&F->AT,2,2,(float *)I->AT_data);
	mat_trans(&F->A, &F->AT);
	mat_init(&F->HT,2,2,(float *)I->HT_data);
	mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}

//二阶卡尔曼滤波器计算
//F：卡尔曼结构体,用于计算矩阵及储存滤波结果；
//signal1:待滤波位置量
//signal2:待滤波速度量
//滤波后的数据 F->filtered_value，0是滤波后位置，1是滤波后速度
float* Kalman2_Filter_Calc(kalman2_filter_t *F, float signal1, float signal2)
{
	float TEMP_data[4] = {0, 0, 0, 0};
	float TEMP_data21[2] = {0, 0};
	mat TEMP,TEMP21;

	mat_init(&TEMP,2,2,(float *)TEMP_data);//
	mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

	F->z.pData[0] = signal1;//z(k)
	F->z.pData[1] = signal2;//z(k)

	//1. xhat'(k)= A xhat(k-1)
	mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
	mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
	mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

	mat_inv(&F->K, &F->P);//
	mat_mult(&F->Pminus, &F->HT, &TEMP);//
	mat_mult(&TEMP, &F->P, &F->K);//

	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
	mat_sub(&F->Q, &F->P, &TEMP);//
	mat_mult(&TEMP, &F->Pminus, &F->P);

	F->filtered_value[0] = F->xhat.pData[0];
	F->filtered_value[1] = F->xhat.pData[1];
	return F->filtered_value;
}

float Kalman2_Filter_Calc2(kalman2_filter_t *F, float signal1, float signal2)
{
	float TEMP_data[4] = {0, 0, 0, 0};
	float TEMP_data21[2] = {0, 0};
	mat TEMP,TEMP21;

	mat_init(&TEMP,2,2,(float *)TEMP_data);//
	mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

	F->z.pData[0] = signal1;//z(k)
	F->z.pData[1] = signal2;//z(k)

	//1. xhat'(k)= A xhat(k-1)
	mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
	mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
	mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

	mat_inv(&F->K, &F->P);//
	mat_mult(&F->Pminus, &F->HT, &TEMP);
	mat_mult(&TEMP, &F->P, &F->K);//

	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
	mat_sub(&F->Q, &F->P, &TEMP);//
	mat_mult(&TEMP, &F->Pminus, &F->P);

	F->filtered_value[0] = F->xhat.pData[0];
	return F->filtered_value[0];
}
