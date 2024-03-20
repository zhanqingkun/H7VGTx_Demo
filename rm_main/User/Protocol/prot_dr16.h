#ifndef __PROT_DR16_H
#define __PROT_DR16_H

#include "stm32h7xx.h"

#define DR16_DATA_LEN 18

#define RC_UP 1
#define RC_MI 3
#define RC_DN 2

typedef struct
{
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch5;
	uint8_t sw1;
	uint8_t sw2;
	__packed struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t l;
		uint8_t r;
	} mouse;
	__packed union
	{
		uint16_t key_code;
		__packed struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;//16个键位
		} bit;
	} kb;
} dr16_t;

typedef enum
{
    KB_Q = 0,
    KB_E,
    KB_R,
    KB_F,
    KB_G,
    KB_Z,
    KB_X,
    KB_C,
    KB_V,
    KB_B,
    KB_CTRL
} key_index_e;

extern dr16_t rc;
extern int kb_status[11];

uint8_t dr16_get_data(dr16_t *rc, uint8_t *data);
void keyboard_scanf(uint16_t key, key_index_e key_index);

#endif
