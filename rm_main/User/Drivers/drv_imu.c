#include "drv_imu.h"
#include "string.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

imu_t chassis_imu, gimbal_imu;

/*
 * @brief     Taurus战队imu数据接收函数
 * @param[in] imu: 陀螺仪数据结构体
 * @param[in] data: 数据指针
 * @retval    void
 */
void imu_get_data(imu_t *imu, uint32_t id, uint8_t *data)
{
    float buffer[2];
    memcpy(buffer, data, 8);
    switch(id)
    {
        case IMU_PALSTANCE_ID:
        {
            imu->wy = 1.0f * buffer[0] / 16.384f * PI / 180;
            imu->wz = 1.0f * buffer[1] / 16.384f * PI / 180;//单位LSB，/16.3835f/57.3f 之后变成 rad/s
            break;
        }
        case IMU_ANGLE_ID:
        {
            imu->pit = 1.0f * buffer[0] * PI / 180;
            imu->yaw = 1.0f * buffer[1] * PI / 180;
            break;
        }
        case IMU_ELSE_ID:
        {
            imu->rol = 1.0f * buffer[0] * PI / 180;
            imu->az= -1.0f * buffer[1];
            break;
        }
        default:break;
    }
}
