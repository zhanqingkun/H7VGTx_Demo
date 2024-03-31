#ifndef __DRV_WS2812B_H
#define __DRV_WS2812B_H

#include "stdint.h"

#define RGB_NUM 7

typedef struct
{
    uint8_t r, g, b;
} rgb_color_t;

typedef struct
{
    uint8_t rgb_status[RGB_NUM];
    uint8_t rgb_buffer[RGB_NUM];
    uint8_t light_state;
    rgb_color_t rgb_data[RGB_NUM];
} rgb_t;

extern rgb_t rgb;



#endif
