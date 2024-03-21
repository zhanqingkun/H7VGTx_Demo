#include "prot_dr16.h"
#include "string.h"

#define ABS(x)  ((x)>0?(x):(-(x)))

scale_t scale;
dr16_t rc;
int kb_status[11] = {0};

/*
 * @brief     dr16遥控器数据接收函数
 * @param[in] rc: 遥控器数据结构体
 * @param[in] data: 数据指针
 * @retval    数据正常返回0，异常返回1
 */
uint8_t dr16_get_data(dr16_t *rc, uint8_t *data)
{
    rc->ch1 = (data[0]      | data[1]  << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (data[1] >> 3 | data[2]  << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (data[2] >> 6 | data[3]  << 2 | data[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (data[4] >> 1 | data[5]  << 7) & 0x07FF;
    rc->ch4 -= 1024;
    rc->ch5 = (data[16]     | data[17] << 8) & 0x07FF;
    rc->ch5 -= 1024;
    rc->sw1 = ((data[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (data[5] >> 4) & 0x0003;
    if ((ABS(rc->ch1) > 660) || (ABS(rc->ch2) > 660) || \
        (ABS(rc->ch3) > 660) || (ABS(rc->ch4) > 660) ||
        (rc->sw1 == 0) || (rc->sw2 == 0)) {
        memset(rc, 0, sizeof(dr16_t));
        return 1;
    }
    rc->mouse.x = data[6] | (data[7] << 8);
    rc->mouse.y = data[8] | (data[9] << 8);
    rc->mouse.z = data[10] | (data[11] << 8);
    rc->mouse.l = data[12];
    rc->mouse.r = data[13];
    rc->kb.key_code = data[14] | data[15] << 8;
    return 0;
}

/*
 * @brief     按键扫描，按一次对应状态取反
 * @param[in] key      : 按键信号
 * @param[in] key_index: 按键序号
 * @retval    void
 */
void keyboard_scanf(uint16_t key, key_index_e key_index)
{
    static uint8_t key_press[11] = {0};
    if (key && (key_press[key_index] == 0)) {
        key_press[key_index] = 1;
        kb_status[key_index] = ~kb_status[key_index];
    } else if (key == 0) {
        key_press[key_index] = 0;
    }
}
