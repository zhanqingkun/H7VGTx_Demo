#ifndef __PROT_VISION_H
#define __PROT_VISION_H

#include "stdint.h"

#define VISION_DATA_LEN 23

#define NAN_PROCESS(now, last)      \
    do {                            \
        if (isnan(now)) {           \
            (now) = (last);         \
        } else {                    \
            (last) = (now);         \
        }                           \
    } while (0)

typedef enum
{
    AIMING = 0,
    UNAIMING = 1,
    TAIL_ERROR = 2,
    REPEAT_ERROR = 3
} vision_rx_status_e;

typedef struct
{
    uint32_t repeat_cnt;
    vision_rx_status_e status;
    union
    {
        uint8_t buff[VISION_DATA_LEN];
        __packed struct
        {
            float yaw;
            float pit;
            float dis;
            float tof;
            float pos;
            uint8_t empty;
            uint8_t cnt : 6;
            uint8_t ist_flag :1;
            uint8_t aim_flag :1;
            uint8_t eof;
        } data;
    } rx[2];
    union
    {
        uint8_t buff[21];
        __packed struct
        {
            uint8_t sof;
            float imu_pit;
            float imu_yaw;
            float imu_pit_spd;
            float imu_yaw_spd;
            uint8_t vacancy :1;
            uint8_t camp :1;
            uint8_t aiming_mode :3;
            uint8_t shooter_speed :3;
            uint8_t empty;
            uint8_t eof1;
            uint8_t eof2;
        } data;
    } tx;
} vision_t;

extern vision_t vision;

#endif
