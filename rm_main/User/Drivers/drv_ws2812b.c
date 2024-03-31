#include "drv_ws2812b.h"
//#include "spi.h"

#define brightness 0x05 //亮度 0x00-0xff
const uint8_t code[] = {0xC0, 0xF8};

rgb_t rgb = {0};

void rgb_set_color(uint8_t id, rgb_color_t color)
{
    if (id < RGB_NUM) {
        rgb.rgb_data[id].g = color.g;
        rgb.rgb_data[id].b = color.b;
        rgb.rgb_data[id].r = color.r;
    }
}

void rgb_reflash(uint8_t reflash_num)
{
    uint8_t data_b, data_r, data_g;
    for (int i = 0; i < reflash_num; i++) {
        data_g = rgb.rgb_data[i].g;
        data_r = rgb.rgb_data[i].r;
        data_b = rgb.rgb_data[i].b;
        for (int j = 0; j < 8; j++) {
            rgb.rgb_buffer[24 * i + 7 - i] = code[data_g & 0x01];
            rgb.rgb_buffer[24 * i + 15 - i] = code[data_r & 0x01];
            rgb.rgb_buffer[24 * i + 23 - i] = code[data_b & 0x01];
            data_g >>= 1; data_r >>= 1; data_b >>= 1;
        }
    }
//    while (HAL_DMA_GetState(&hdma_spi1_tx) != HAL_DMA_STATE_READY);
//    HAL_SPI_Transmit_DMA(&hspi1, SPI_RGB_BUFFER, 24);
}
