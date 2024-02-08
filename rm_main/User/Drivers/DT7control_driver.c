#include "DT7control_driver.h"
#include "string.h"

#define ABS(x)  ((x)>0?(x):(-(x)))

rc_t rc;

//接收DT7遥控器数据
//数据错误时返回1 重新进行一次空闲中断来对齐数据
uint8_t DT7control_Receive(rc_t *rc, uint8_t *buff)
{
	rc->ch1 = (buff[0]      | buff[1]  << 8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1] >> 3 | buff[2]  << 5) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2] >> 6 | buff[3]  << 2 | buff[4] << 10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4] >> 1 | buff[5]  << 7) & 0x07FF;
	rc->ch4 -= 1024;
	rc->ch5 = (buff[16]     | buff[17] << 8) & 0x07FF;
	rc->ch5 -= 1024;
	rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	rc->sw2 = (buff[5] >> 4) & 0x0003;
	if ((ABS(rc->ch1) > 660) || (ABS(rc->ch2) > 660) || \
		(ABS(rc->ch3) > 660) || (ABS(rc->ch4) > 660) ||
		(rc->sw1 == 0) || (rc->sw2 == 0))
	{
		memset(rc, 0, sizeof(rc_t));
		return 1;
	}
	rc->mouse.x = buff[6] | (buff[7] << 8);
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10] | (buff[11] << 8);
	rc->mouse.l = buff[12];
	rc->mouse.r = buff[13];
	rc->kb.key_code = buff[14] | buff[15] << 8;
	return 0;
}
