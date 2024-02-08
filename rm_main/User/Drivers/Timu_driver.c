#include "Timu_driver.h"
#include "can_comm.h"
#include "fdcan.h"
#include "string.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

Timu_t imu;

//接收Taurus战队imu数据
void Timu_Receive(Timu_t* imu, uint32_t id, uint8_t* rx_data)
{
	//底盘IMU pit抬头为正 yaw顺时针为正 roll从车尾向前看 顺时针为正
	float buffer[2];
	memcpy(buffer, rx_data, 8);
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
			imu->pitch = 1.0f * buffer[0] * PI / 180;
			imu->yaw   = 1.0f * buffer[1] * PI / 180;
			break;
		}
		case IMU_ELSE_ID:
		{
			imu->roll = 1.0f * buffer[0] * PI / 180;
			imu->az= -1.0f * buffer[1];
			break;
		}
		default:break;
	}
}
