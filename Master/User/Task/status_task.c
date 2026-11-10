#include "status_task.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "my_main.h"
/* BR has no TR chassis/world-path task. Motor feedback watchdogs run in Upper_TxTask. */
void Status_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        if (BspCan_HasBusOff())
        {
            int32_t lock = osKernelLock();
            (void)BspCan_Recover();
            if (lock >= 0)
                (void)osKernelRestoreLock(lock);
        }
        osDelay((osKernelGetTickFreq() * 2U + 999U) / 1000U);
    }
}
