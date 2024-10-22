#include "comm_task.h"
#include "cmsis_os.h"
#include "prot_vision.h"
#include "prot_power.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"

void comm_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        dji_motor_output_data();
        ht_motor_output_data();
        vision_output_data();
        power_output_data();
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
