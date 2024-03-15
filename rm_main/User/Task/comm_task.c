#include "comm_task.h"
#include "cmsis_os.h"
#include "drv_dji_motor.h"
#include "drv_ht_motor.h"

void comm_task(void const* argument)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        dji_motor_output_data();
        ht_motor_output_data();
//        taskENTER_CRITICAL();
//        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
