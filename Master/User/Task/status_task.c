/*
 * @file status_task.c
 * @brief status_task模块的用户接口与实现。
 */

#include "status_task.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "my_main.h"

/* 功能：执行 Status_Task 函数对应的模块操作。 */
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
