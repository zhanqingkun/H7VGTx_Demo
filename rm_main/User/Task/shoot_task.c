#include "shoot_task.h"
#include "mode_switch_task.h"
#include "control_def.h"
#include "prot_judge.h"
#include "prot_dr16.h"
#include "cover_ctrl.h"
#include "fric_ctrl.h"
#include "trigger_ctrl.h"
#include "data_buffer.h"
#include "cmsis_os.h"

#define SHOOT_SPEED_NUM 15
#define ABS(x) ((x>0)? (x): (-(x)))

shoot_t shoot;
//static buffer_t *shoot_speed_buffer;

static void shoot_param_update(void)
{
    //更新裁判系统数据
    if (robot_status.shooter_barrel_heat_limit != 0) {
        shoot.shoot_speed_ref = 30;//射速上限
        shoot.barrel.heat_max = robot_status.shooter_barrel_heat_limit;//枪管热量上限
        shoot.barrel.cooling_rate = robot_status.shooter_barrel_cooling_value;//枪管冷却速率
    }
    //更新模拟裁判系统数据
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //当前枪管(理论)热量
    if (shoot.barrel.heat < 0) shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //当前枪管(理论)剩余热量
}

static void shoot_test(void) {
//    static float last_shoot_speed;
//    shoot.barrel.bullet_spd.fdb = shoot_data.initial_speed;
//    if (ABS(shoot.barrel.bullet_spd.fdb - last_shoot_speed) > 1e-5f) {
//        ++shoot.barrel.shoot_cnt;//统计发射子弹数
//        buffer_put_noprotect(shoot_speed_buffer, &shoot.barrel.bullet_spd.fdb);
//        float vision_data_array[SHOOT_SPEED_NUM] = {0};
//        uint32_t real_num = ubf_pop_into_array_new2old(shoot_speed_buffer, vision_data_array, 0, SHOOT_SPEED_NUM);
//        arm_var_f32(vision_data_array, real_num, &shoot.barrel.bullet_spd.std);  //计算数据方差
//        shoot.barrel.bullet_spd.std = sqrt(shoot.barrel.bullet_spd.std);
//        last_shoot_speed = shoot.barrel.bullet_spd.fdb;
//    }
}

static void shoot_mode_sw(void)
{
    /* 系统历史状态机 */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;

    /* 更新裁判系统参数 */
    shoot_param_update();

    /* 模式切换 */
    switch (ctrl_mode) {
        case PROTECT_MODE: {
            fric.mode = FIRC_MODE_STOP;
            shoot.trigger_mode = TRIGGER_MODE_PROTECT;
            house_mode= HOUSE_MODE_PROTECT;
            break;
        }
        
        case REMOTER_MODE: {
            /* 摩擦轮和拨盘模式切换 */
            switch (rc.sw2) {
                case RC_UP: {
                    fric.mode = FIRC_MODE_STOP;
                    shoot.trigger_mode = TRIGGER_MODE_STOP;
                    break;
                }
                case RC_MI: {
                    if (fric.init_flag) {
                        fric.mode = FIRC_MODE_RUN;  //开启摩擦轮
//                        fric.mode = FIRC_MODE_STOP;  //开启摩擦轮
                    }
                    shoot.trigger_mode = TRIGGER_MODE_STOP;
                    break;
                }
                case RC_DN: {
                    fric.mode = FIRC_MODE_STOP;
                    shoot.trigger_mode = TRIGGER_MODE_SERIES;
//                    shoot.trigger_mode = TRIGGER_MODE_STOP;
                    if (fric.init_flag) {
                        fric.mode = FIRC_MODE_RUN;  //开启摩擦轮
//                        fric.mode = FIRC_MODE_STOP;  //开启摩擦轮
                    }
//                    if (rc_fsm_check(RC_RIGHT_LU)) {  //遥控器上电前，左拨杆置右上
//                        shoot.trigger_mode = TRIGGER_MODE_SINGLE;  //单发 遥控器单发不能开底盘
//                    } else {
//                        shoot.trigger_mode = TRIGGER_MODE_SERIES;  //连发
//                    }
                    break;
                }
                default: break;
            }
            /* 弹舱盖模式切换 */
            static uint8_t house_switch_enable = 1;
            if (last_ctrl_mode != REMOTER_MODE) {
                house_mode = HOUSE_MODE_CLOSE;
            }
            if (ABS(rc.ch5) < 10) {
                house_switch_enable = 1;
            }
            if (house_switch_enable && rc.ch5 <= -500) {  //切换弹舱开关状态标志位
                house_switch_enable = 0;
                house_mode = (house_mode_e)(!(uint8_t)house_mode);  //开关弹舱盖
            }
            break;
        }
        case VISION_MODE:
        case KEYBOARD_MODE: {
            /* 摩擦轮模式切换 */
            if (robot_status.power_management_shooter_output && fric.init_flag) {  //发射机构得到供电 
                fric.mode = FIRC_MODE_RUN;  //开关摩擦轮         
            } else {
                fric.mode = FIRC_MODE_STOP;  //摩擦轮断电，软件保护，禁用摩擦轮
            }
            
            /* 拨盘模式切换 */
            if (fric.mode == FIRC_MODE_RUN) {  //开摩擦轮
//                if (vision.mode == vMODE_bENERGY || vision.mode == vMODE_sENERGY) {  //能量机关 单发模式
//                    shoot.trigger_mode = TRIGGER_MODE_SINGLE;
//                } else {  //其他模式 连发模式
                    shoot.trigger_mode = TRIGGER_MODE_SERIES;
//                }
            } else {
                shoot.trigger_mode = TRIGGER_MODE_STOP;  //
            }
            
            /* 弹舱盖模式切换 */
            key_scan(KEY_SHOOT_HOUSE);
            if (last_ctrl_mode != KEYBOARD_MODE) {  //首次进入键盘模式 关闭弹舱盖
                house_mode = HOUSE_MODE_CLOSE;
                key_status_clear(KEY_SHOOT_HOUSE);
            }
            if (kb_status[KEY_SHOOT_HOUSE] == KEY_RUN) {
                house_mode = HOUSE_MODE_OPEN;  //按过奇数次辅助按键， 打开弹舱盖
            } else if (kb_status[KEY_SHOOT_HOUSE] == KEY_END) {
                house_mode = HOUSE_MODE_CLOSE;  //按过偶数次辅助按键， 关闭弹舱盖
            }
            break;
        }
        default: break;
    }
    /* 历史状态更新 */
    last_ctrl_mode = ctrl_mode;
}

static void shoot_init(void)
{
    memset(&shoot, 0, sizeof(shoot_t));
    //发射器底层初始化
    fric_init();   //摩擦轮初始化 在任务中控制初始化
    trigger_init();//拨盘初始化
    house_init();       //弹舱初始化
    //发射器模式初始化
    fric.mode           = FIRC_MODE_STOP;
    shoot.trigger_mode  = TRIGGER_MODE_PROTECT;
    house_mode          = HOUSE_MODE_PROTECT;  //上电保护，弹舱盖无力
    //枪管参数初始化
    shoot.trigger_period = TRIGGER_PERIOD;
    shoot.barrel.cooling_rate   = 10;
    shoot.barrel.heat_max       = 50;
    shoot.shoot_speed_ref       = 30;
    //历史射速反馈缓存区
//    shoot_speed_buffer = buffer_create(SHOOT_SPEED_NUM, sizeof(float));
}

void shoot_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    shoot_init();
    for(;;)
    {
//        taskENTER_CRITICAL();
        /* 电调初始化 */
        static uint8_t last_fric_enable, fric_enable;
        fric_enable = robot_status.power_management_shooter_output;
        if (fric_enable && !last_fric_enable) {
            fric.init_cnt = 0;
            fric.init_flag = 0;
        }
        last_fric_enable = fric_enable;
        fric_init();
        shoot_mode_sw();    /* 发射器模式切换 */
        fric_control();	/* 摩擦轮电机控制 */
        house_control();    /* 弹舱盖控制 */
        trigger_control();	/* 拨弹电机控制 */
        shoot_test();
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
