#include "shoot_task.h"
#include "cmsis_os.h"

void shoot_task(void const *argu)
{

    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {

        osDelayUntil(&thread_wake_time, 2);
    }
}
