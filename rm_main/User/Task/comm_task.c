#include "comm_task.h"
#include "cmsis_os.h"
#include "prot_vision.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"

uint32_t temp_cnt;
void comm_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        temp_cnt++;
        dji_motor_output_data();
        ht_motor_output_data();
        vision_output_data();
        power_output_data();
        osDelayUntil(&thread_wake_time, 2);
    }
}
