#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stdint.h"

typedef enum
{
    FIRC_MODE_STOP = 0,
    FIRC_MODE_RUN = 1
} firc_mode_e;

typedef struct {
    firc_mode_e mode;
    uint32_t init_cnt;
    uint8_t init_flag;
    uint16_t low_pwm;
    uint16_t mid_pwm;
    uint16_t high_pwm;
} fric_t;

extern fric_t fric;
uint8_t fric_init(void);
void fric_control(void);

#endif
