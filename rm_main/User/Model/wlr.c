#include "wlr.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "drv_dji_motor.h"
#include "pid.h"
#include "kalman_filter.h"
#include "math_lib.h"
#include "math_matrix.h"

#define WLR_SIGN(x) ((x) > 0? (1): (-1))

#define CHASSIS_PERIOD_DU 2

const float LegLengthParam[5] = {0.150f, 0.270f, 0.270f, 0.150f, 0.150f};
float mb = 4.4f, ml = 2.09f, mw = 0.715f;//机体质量 腿部质量 轮子质量 14.5
const float BodyWidth = 0.42f;//两轮间距
const float WheelRadius = 0.06f;//轮子半径
const float LegLengthMax = 0.30f, LegLengthMin = 0.15f;

const float LegLengthJump1 = 0.15f;//压腿
const float LegLengthJump2 = 0.35f;//蹬腿
const float LegLengthJump3 = 0.24f;//收腿
const float LegLengthJump4 = 0.22f;//落地
const float LegLengthFly = 0.20f;//腾空
const float LegLengthHigh = 0.20f;//长腿
const float LegLengthNormal = 0.15f;//正常

//float x3_balance_zero = 0.03, x5_balance_zero = -0.035f;//腿摆角角度偏置 机体俯仰角度偏置
float x3_balance_zero = 0.06f, x5_balance_zero = -0.02f;//腿摆角角度偏置 机体俯仰角度偏置

//								位移  速度	角度	角速度  角度	角速度
float K_Array_Wheel[2][6] =		{{60, 30, 80, 8, 300, 10}, 
                                { -0, -0.7, -8, -1, 3, 2}};
float K_Array_Leg[2][6] =		{{6.08535, 12.6636, 49.1418, 7.57203, 54.8073, 12.7387}, {-0.793527, -1.75976, -13.1544, -1.78789, 4.8796, 1.66868}};
float K_Array_Fly[2][6] =		{{0, 0, 80, 10, 0, 0}, { 0, 0, 0, 0, 0, 0}};

float K_Array_test[2][6];
float K_Array_List[12][4] = 
//{{6.33331,37.1637,-173.267,195.695},{13.375,62.6818,-306.463,349.224},{14.3574,523.661,-1613.6,1529.67},{4.56946,50.9723,-144.236,129.666},{32.2546,24.3717,604.921,-950.859},{9.25451,-6.78675,173.053,-249.371},{-0.907961,1.91211,-9.19263,11.1016},{-2.00572,5.03316,-22.1698,25.8241},{-4.53142,-42.3224,36.5561,-9.40343},{-0.980517,-1.26602,-10.414,10.7121},{13.699,-30.8227,43.8683,-31.5937},{2.94063,-5.26768,6.22139,-3.89125}};    
//{{7.7883,46.688,-215.454,242.688},{15.0481,71.2191,-344.839,392.009},{16.0739,537.105,-1644.75,1553.31},{4.70938,54.5476,-149.965,133.102},{29.3234,27.6173,614.168,-967.687},{8.55949,-6.85075,177.838,-255.908},{-1.11401,2.46758,-11.7282,14.1079},{-2.26846,5.99326,-26.0743,30.2096},{-4.74503,-41.8374,32.0378,-4.75168},{-1.0219,-1.15369,-11.5588,11.717},{14.0627,-32.6827,49.9302,-38.2469},{3.02982,-5.67978,7.63334,-5.45232}};
{{7.00681,41.1423,-185.674,205.977},{15.4159,74.7049,-349.699,390.408},{14.0718,685.814,-2051.,1902.38},{5.08813,65.8114,-175.561,152.294},{18.566,-19.1885,902.991,-1325.06},{6.46489,-20.1441,248.876,-340.781},{-0.730768,2.12264,-9.87338,11.7523},{-1.68527,5.33373,-23.4529,27.2529},{-4.39641,-36.0867,18.7682,8.33676},{-0.929593,-0.55816,-11.7229,12.0788},{12.6942,-29.997,50.1496,-42.3926},{2.70374,-5.18173,7.91857,-6.59155}};
    
float K_Array_List_lq[12][4][4] = 
{{{5.22798,0.120698,-0.000840624,1.30043e-6},{-27.8965,0.476016,-0.00280186,0},{-63.4158,0.0760932,0,0},{116.423,0,0,0}},
{{8.71734,0.29992,-0.00203995,2.86743e-6},{-71.0269,0.76054,-0.00457909,0},{-15.2343,0.168384,0,0},{71.7744,0,0,0}},
{{-22.99,1.69686,-0.0119447,0.0000182525},{-99.7898,4.6908,-0.0268931,0},{-761.51,0.472008,0,0},{1096.43,0,0,0}},
{{5.83449,0.0891018,-0.000731637,1.77972e-6},{-48.1618,0.816036,-0.00471951,0},{31.5375,0.0972437,0,0},{-53.7863,0,0,0}},
{{75.0024,-1.07961,0.00679659,-6.20881e-6},{144.783,1.05686,-0.00505925,0},{-328.529,-0.3575,0,0},{217.495,0,0,0}},
{{15.5352,-0.311456,0.00198484,-2.0315e-6},{75.1284,0.202374,-0.000786395,0},{-187.928,-0.148803,0,0},{181.46,0,0,0}},
{{-1.33258,0.0159886,-0.000100859,9.22646e-8},{-1.41807,-0.0127862,0.0000605034,0},{1.05816,0.00470869,0,0},{2.29189,0,0,0}},
{{-2.72025,0.0238647,-0.000148877,1.18866e-7},{-0.375646,0.0154076,-0.0000903077,0},{-14.2395,0.00189886,0,0},{26.0355,0,0,0}},
{{-4.91795,-0.0760693,0.00086145,-3.06738e-6},{54.8722,-2.12031,0.0117915,0},{77.9843,-0.0406164,0,0},{-84.3636,0,0,0}},
{{-2.00836,0.00600775,-0.0000267199,-5.19685e-8},{-0.986647,0.118254,-0.000643844,0},{-33.8391,-0.00886079,0,0},{46.0011,0,0,0}},
{{4.86264,0.0699955,-0.00050198,8.93186e-7},{-33.212,0.353139,-0.00210746,0},{12.5199,0.0682708,0,0},{0.422155,0,0,0}},
{{2.93993,0.00177889,-0.0000390133,2.43053e-7},{-14.3874,0.104903,-0.000639501,0},{20.4728,0.026013,0,0},{-23.4521,0,0,0}}};

//{{{1.33415,0.101041,-0.000763204,1.44975e-6},{26.434,0.804797,-0.00453317,0},{-343.25,0.0413357,0,0},{500.981,0,0,0}},
//{{-3.90396,0.424048,-0.00291835,4.0665e-6},{23.03,1.19914,-0.00684435,0},{-480.307,0.108,0,0},{715.109,0,0,0}},
//{{-66.5428,2.2886,-0.0172469,0.0000321737},{37.7167,9.2713,-0.0519865,0},{-1974.19,0.39766,0,0},{2643.94,0,0,0}},
//{{-3.35566,0.212931,-0.00163253,3.18022e-6},{-5.88127,1.29444,-0.00724296,0},{-205.008,0.0500396,0,0},{258.261,0,0,0}},
//{{183.708,-3.76028,0.0235987,-0.0000196485},{-106.095,6.06991,-0.0330616,0},{266.928,-0.293359,0,0},{-849.386,0,0,0}},
//{{40.3007,-0.774233,0.00494746,-4.70779e-6},{-9.78313,0.56953,-0.00295263,0},{155.896,-0.103568,0,0},{-320.958,0,0,0}},
//{{-1.27857,0.00853655,-0.0000561747,6.39445e-8},{2.22789,0.00825126,-0.000049683,0},{-15.2406,0.00209506,0,0},{23.3951,0,0,0}},
//{{-1.99724,-0.00482709,0.0000302318,-2.72361e-8},{7.37913,0.0372337,-0.000202982,0},{-44.939,-0.00156445,0,0},{65.3821,0,0,0}},
//{{-2.34702,-0.0953687,0.00108506,-3.88064e-6},{61.5999,-2.24797,0.0124891,0},{72.5251,-0.0375582,0,0},{-75.3194,0,0,0}},
//{{-1.11389,-0.000677846,0.000017886,-9.81005e-8},{-0.655812,0.0306278,-0.000167317,0},{-20.1544,-0.00409871,0,0},{27.8306,0,0,0}},
//{{6.91722,0.23308,-0.00158393,2.22708e-6},{-96.3639,0.556565,-0.0033791,0},{199.92,0.136527,0,0},{-244.656,0,0,0}},
//{{2.31978,0.0256059,-0.000188478,3.60758e-7},{-15.6933,0.108086,-0.000654082,0},{27.5635,0.0256094,0,0},{-34.3077,0,0,0}}};

//{{{1.1984,0.122238,-0.000926183,1.77661e-6},{19.44,1.08297,-0.00609866,0},{-370.2,0.0545384,0,0},{538.101,0,0,0}},
//{{-6.88354,0.559844,-0.00386205,5.42629e-6},{7.02166,1.72713,-0.0098291,0},{-528.705,0.141218,0,0},{784.653,0,0,0}},
//{{-100.53,3.24203,-0.0242823,0.0000444897},{24.409,12.2726,-0.0687939,0},{-2463.38,0.521965,0,0},{3282.95,0,0,0}},
//{{-5.52704,0.285099,-0.00222078,4.4922e-6},{-22.2434,2.03687,-0.01137,0},{-265.242,0.0624373,0,0},{334.062,0,0,0}},
//{{233.084,-5.33491,0.0336474,-0.0000288727},{-167.561,7.71501,-0.042163,0},{447.04,-0.327525,0,0},{-1211.81,0,0,0}},
//{{50.5033,-1.09816,0.00702321,-6.67241e-6},{-21.8237,0.745852,-0.00390794,0},{223.066,-0.120536,0,0},{-437.261,0,0,0}},
//{{-1.16409,0.0104186,-0.0000684108,7.69156e-8},{2.20093,0.00926133,-0.0000560176,0},{-15.3571,0.00249728,0,0},{23.5159,0,0,0}},
//{{-1.86154,0.000894375,-5.27695e-6,1.09446e-9},{7.38306,0.0257976,-0.000142475,0},{-43.2775,-0.000167581,0,0},{63.2804,0,0,0}},
//{{-2.76663,-0.0789309,0.000973869,-3.74231e-6},{68.7855,-2.30631,0.0128171,0},{57.6264,-0.0391855,0,0},{-56.103,0,0,0}},
//{{-1.24159,0.00427786,-0.0000124789,-7.71814e-8},{1.01367,-0.00097185,6.9019e-6,0},{-20.1301,-0.00346055,0,0},{28.0976,0,0,0}},
//{{6.8356,0.208006,-0.00142721,2.0927e-6},{-96.3514,0.633761,-0.00379421,0},{195.977,0.131397,0,0},{-243.247,0,0,0}},
//{{2.26419,0.0206838,-0.000157744,3.34521e-7},{-16.1278,0.130845,-0.000777772,0},{27.3999,0.0246009,0,0},{-34.879,0,0,0}}};


wlr_t wlr;
lqr_t lqr[2];

kalman_filter_t kal_fn[2];

pid_t pid_leg_length[2];
pid_t pid_leg_length_fast[2];
pid_t pid_q0, pid_roll, pid_yaw, pid_wz;

static float wlr_fn_calc(float az, float Fy_fdb, float T0_fdb, float L0[3], float theta[3])
{
    float Fwy = Fy_fdb * cosf(theta[0]) + T0_fdb * sinf(theta[0]) / L0[0];//轮子受到腿部机构竖直方向的作用力
    float yw_ddot = az
                    - L0[2] * cosf(theta[0])
                    + 2 * L0[1] * theta[1] * sinf(theta[0])
                    + L0[0] * theta[2] * sinf(theta[0])
                    + L0[0] * powf(theta[1], 2) * cosf(theta[0]);//轮子竖直方向的加速度
    return Fwy + mw * GRAVITY + mw * yw_ddot;
}

static void k_array_fit1(float K[2][6], float high_fdb)
{
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 6; j++) {
            K[i][j] = 0;
            for(int h = 0; h < 4; h++) {
                K[i][j] += K_Array_List[i * 6 + j][h] * powf(high_fdb, h);
            }
        }
    }
}

static void k_array_fit2(float K[2][6], float high_fdb, float q0_fdb)
{
    float temp;
    for(int i = 0; i < 2; i++)
        for(int j = 0; j < 6; j++) {
            temp = 0;
            for(int x = 0; x <= 3; x++)
                for(int y = 0; y <= 3; y++)
                    temp += (K_Array_List_lq[i * 6 + j][x][y] * powf(high_fdb, x) * powf(q0_fdb, y));
            K[i][j] = temp;
        }
}

void wlr_init(void)
{
	wlr.max_stop_angle = 10;
	wlr.max_wz_error = 0.8f;
	wlr.high_set = LegLengthNormal;
	wlr.roll_set = 0;
	wlr.q0_set = PI / 2;
	
	twm_init(&twm, BodyWidth, WheelRadius);
	tlm_init(&tlm, LegLengthMax, LegLengthMin, BodyWidth);
	for(int i = 0; i < 2; i++)
	{
		//腿部长度初始化
		vmc_init(&vmc[i], LegLengthParam);
		//卡尔曼滤波器初始化
        kalman_filter_init(&kal_fn[i], 1, 0, 1);
        kal_fn[i].A_data[0] = 1;
        kal_fn[i].H_data[0] = 1;
        kal_fn[i].Q_data[0] = 1;
        kal_fn[i].R_data[0] = 100;
		//PID参数初始化
		pid_init(&pid_leg_length[i], 500, 0, 20000, 10, 20);//i 2.5f
		pid_init(&pid_leg_length_fast[i], 1000, 0, 10000, 30, 50);
	}
	//卡尔曼滤波器初始化
	
	//PID参数初始化
//    pid_init(&pid_yaw, -7, 0, 0, 0, 10);
//	pid_init(&pid_wz, 2.0f, 0, 7.0f, 0, 2.5f);
    pid_init(&pid_yaw, -5, 0, 0, 0, 7);
	pid_init(&pid_wz, 2.0f, 0, 7.0f, 0, 2.5f);//与LQR的速度控制协同
	pid_init(&pid_q0, 60, 0, 100, 0, 10);//与LQR的虚拟腿摆角控制拮抗 60 0 120
	pid_init(&pid_roll, 500, 0, 4000, 0, 15);//与VMC的腿长控制协同  1000 0 3500
}

void wlr_protest(void)
{
	pid_leg_length[0].i_out = 0;
	pid_leg_length[1].i_out = 0;
}

//轮子：位移、速度   摆角：角度、角速度   机体俯仰：角度、角速度
void wlr_control(void)
{
	//------------------------反馈数据更新------------------------//
	wlr.wz_set = pid_calc(&pid_yaw, wlr.yaw_fdb + wlr.yaw_err, wlr.yaw_fdb);
	//更新两轮模型
	twm_feedback_calc(&twm, wlr.side[0].wy, wlr.side[1].wy, wlr.wz_fdb);//输入左右轮子转速
	twm_reference_calc(&twm, wlr.v_set, wlr.wz_set);//计算两侧轮腿模型的设定速度
	//两侧轮腿分别更新数据
	for(int i = 0; i < 2; i++) {
		//更新腿部VMC模型
		vmc_forward_solution(&vmc[i], wlr.side[i].q1, wlr.side[i].q4, wlr.side[i].w1, \
									  wlr.side[i].w4, wlr.side[i].t1, wlr.side[i].t4);
		//LQR输入反馈值
		lqr[i].last_x2 = lqr[i].X_fdb[1];
		lqr[i].X_fdb[1] = -wlr.side[i].wy * WheelRadius;
		lqr[i].X_fdb[0] += (lqr[i].X_fdb[1] + lqr[i].last_x2) / 2 * CHASSIS_PERIOD_DU * 0.001f;//使用梯形积分速度求位移
		lqr[i].X_fdb[4] = x5_balance_zero + wlr.pit_fdb;
		lqr[i].X_fdb[5] = wlr.wy_fdb;
		lqr[i].X_fdb[2] = x3_balance_zero + (PI / 2 - lqr[i].X_fdb[4] - vmc[i].q_fdb[0]);
		lqr[i].X_fdb[3] = lqr[i].X_fdb[5] - vmc[i].V_fdb.e.w0_fdb;
		lqr[i].dot_x4 = (lqr[i].X_fdb[3] - lqr[i].last_x4) / (CHASSIS_PERIOD_DU * 0.001f); //腿倾角加速度(状态变量x4的dot)计算
		lqr[i].last_x4 = lqr[i].X_fdb[3];
		data_limit(&lqr[i].X_fdb[0], 0.5f, -0.5f);//位移限幅  位移系数主要起到一个适应重心的作用 不用太大
        if(ABS(wlr.v_set) > 1e-3f || ABS(wlr.wz_set) > 0.1f || ABS(vmc[0].q_fdb[0] - vmc[1].q_fdb[0]) > 0.01f)//有输入速度 或 两腿有差角时 将位移反馈置0  不发挥作用
			lqr[i].X_fdb[0] = 0;
		//支持力解算
		float L0_array[3] = {vmc[i].L_fdb, vmc[i].V_fdb.e.vy0_fdb, vmc[i].Acc_fdb.L0_ddot};
		float theta_array[3] = {lqr[i].X_fdb[2], lqr[i].X_fdb[3], lqr[i].dot_x4};
		wlr.side[i].Fn_fdb = wlr_fn_calc(wlr.az_fdb, vmc[i].F_fdb.e.Fy_fdb, vmc[i].F_fdb.e.T0_fdb, L0_array, theta_array);
        kal_fn[i].measured_vector[0] = wlr.side[i].Fn_fdb;
        kalman_filter_update(&kal_fn[i]);
        wlr.side[i].Fn_kal = kal_fn[i].filter_vector[0];
		//离地检测
		if(wlr.side[i].Fn_kal < 17.0f)
			wlr.side[i].fly_cnt++;
		else if(wlr.side[i].fly_cnt != 0)
			wlr.side[i].fly_cnt--;
        if(wlr.side[i].fly_cnt > 30) {
            wlr.side[i].fly_cnt = 30;
            wlr.side[i].fly_flag = 1;
        } else if(wlr.side[i].fly_cnt == 0)
            wlr.side[i].fly_flag = 0;
	}
	//高度选择 跳跃状态改变
	if (wlr.jump_flag == 1) {//跳跃起跳状态 先压腿
		wlr.high_set = LegLengthJump1;
		if(vmc[0].L_fdb < LegLengthJump1 && vmc[1].L_fdb < LegLengthJump1)
			wlr.jump_flag = 2;
	} else if (wlr.jump_flag == 2) {//起跳 弹腿
		wlr.high_set = LegLengthJump2;
		if(vmc[0].L_fdb > LegLengthJump2 && vmc[1].L_fdb > LegLengthJump2)
			wlr.jump_flag = 3;
	} else if (wlr.jump_flag == 3) {//收腿
		wlr.high_set = LegLengthJump3;
		if (vmc[0].L_fdb < LegLengthJump3 && vmc[1].L_fdb < LegLengthJump3 && !wlr.side[0].fly_flag && !wlr.side[1].fly_flag)
			wlr.jump_flag = 4;
	} else if (wlr.jump_flag == 4) {//落地
		wlr.high_set = LegLengthJump4;
		wlr.jump_cnt++;
		if (wlr.jump_cnt > 200) {
			wlr.jump_flag = 0;
			wlr.jump_cnt = 0;
		}
	} else if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
		wlr.high_set = LegLengthFly;
    } else {
		if (wlr.high_flag || wlr.ctrl_mode == 2)//大高度
            wlr.high_set = LegLengthHigh;
        else
            wlr.high_set = LegLengthNormal;
    }
	//更新两腿模型
	tlm_gnd_roll_calc(&tlm, -wlr.roll_fdb, vmc[0].L_fdb, vmc[1].L_fdb);//计算地形倾角
	if (wlr.jump_flag != 0 || (wlr.side[0].fly_flag && wlr.side[1].fly_flag))
		tlm.l_ref[0] = tlm.l_ref[1] = wlr.high_set;
	else
		tlm_leg_length_cacl(&tlm, wlr.high_set, 0);//计算腿长设定值
	//------------------------状态选择------------------------//
	//根据当前状态选择合适的控制矩阵
	if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {//腾空
		aMartix_Cover(lqr[0].K, (float*)K_Array_Fly, 2, 6);
		aMartix_Cover(lqr[1].K, (float*)K_Array_Fly, 2, 6);
	} else if (wlr.side[0].fly_flag == 0 && wlr.side[1].fly_flag == 0) {//在地面
        if (wlr.ctrl_mode == 2) {//力控
//            k_array_fit1(K_Array_test, vmc[0].L_fdb);
//            aMartix_Cover(lqr[0].K, (float*)K_Array_test, 2, 6);
//            k_array_fit1(K_Array_test, vmc[1].L_fdb);
//            aMartix_Cover(lqr[1].K, (float*)K_Array_test, 2, 6);
            k_array_fit2(K_Array_test, vmc[0].L_fdb, vmc[0].q_fdb[0]/PI*180);
            aMartix_Cover(lqr[0].K, (float*)K_Array_test, 2, 6);
            k_array_fit2(K_Array_test, vmc[1].L_fdb, vmc[1].q_fdb[0]/PI*180);
            aMartix_Cover(lqr[1].K, (float*)K_Array_test, 2, 6);
		} else if (wlr.ctrl_mode == 1) {//位控
			aMartix_Cover(lqr[0].K, (float*)K_Array_Wheel, 2, 6);
			aMartix_Cover(lqr[1].K, (float*)K_Array_Wheel, 2, 6);
		}
	}
	//------------------------控制数据更新------------------------//
	//全身运动控制
	wlr.q0_offs   = pid_calc(&pid_q0, vmc[0].q_fdb[0], vmc[1].q_fdb[0]);//双腿摆角同步控制
	wlr.roll_offs = pid_calc(&pid_roll, wlr.roll_set, wlr.roll_fdb);
//	if (wlr.wz_set > wlr.wz_fdb + wlr.max_wz_error)//z轴速度窗口 很重要
//		wlr.wz_set = wlr.wz_fdb + wlr.max_wz_error;
//	else if (wlr.wz_set < wlr.wz_fdb - wlr.max_wz_error)
//		wlr.wz_set = wlr.wz_fdb - wlr.max_wz_error;
	wlr.wz_offs   = pid_calc(&pid_wz, wlr.wz_fdb, wlr.wz_set);//Yaw控制
	//两侧轮腿分别独立控制
	for (int i = 0; i < 2; i++) {
		//LQR输入控制值 计算得出轮子与腿的力矩
        //速度融合 在快要撞限位时减小速度控制
        if (wlr.side[i].q1 > 3.6f)
            lqr[i].X_ref[1] = data_fusion(twm.v_ref[i], lqr[i].X_fdb[1], (wlr.side[i].q1 - 3.6f)/0.3f);
        else if (wlr.side[i].q4 < -0.3f)
            lqr[i].X_ref[1] = data_fusion(twm.v_ref[i], lqr[i].X_fdb[1], (-0.3f - wlr.side[i].q4)/0.3f);
        else
            lqr[i].X_ref[1] = twm.v_ref[i];
		aMartix_Add(1, lqr[i].X_ref, -1, lqr[i].X_fdb, lqr[i].X_diff, 6, 1);
		aMartix_Mul(lqr[i].K, lqr[i].X_diff, lqr[i].U_ref, 2, 6, 1);
		//腿部虚拟力控制
		if (wlr.jump_flag == 2 || wlr.jump_flag == 4)						//跳跃蹬腿阶段 响应要大
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb)\
								+ mb * GRAVITY / 2 + WLR_SIGN(i) * wlr.roll_offs;
		else if (wlr.jump_flag == 3)                                        //跳跃收腿阶段 响应要大
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb);
		else if (wlr.side[0].fly_flag && wlr.side[1].fly_flag) {            //浮空收腿 响应不用那么大
			wlr.side[i].Fy = pid_calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb) - ml * GRAVITY;
			wlr.wz_offs = 0;
		} else																//常态 跳跃压腿阶段 跳跃落地阶段
			wlr.side[i].Fy = pid_calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)\
                                + 35 * vmc[i].L_fdb + 19 + WLR_SIGN(i) * wlr.roll_offs;
		wlr.side[i].T0 = -lqr[i].U_ref[0] / 2 + WLR_SIGN(i) * wlr.q0_offs;	//两条腿时，LQR输出力矩需要除以2
		vmc_inverse_solution(&vmc[i], wlr.high_set, wlr.q0_set, wlr.side[i].T0, wlr.side[i].Fy);
	}
	//------------------------控制数据输出------------------------//
	for (int i = 0; i < 2; i++) {
		wlr.side[i].T1 =  vmc[i].T_ref.e.T1_ref;
		wlr.side[i].T4 =  vmc[i].T_ref.e.T4_ref;
		wlr.side[i].Tw = -lqr[i].U_ref[1] + WLR_SIGN(i) * wlr.wz_offs;
		wlr.side[i].P1 =  vmc[i].q_ref[1];
		wlr.side[i].P4 =  vmc[i].q_ref[4];
	}
}
