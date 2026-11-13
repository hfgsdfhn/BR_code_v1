/*
 * @file status_task.c
 * @brief status_task模块的用户接口与实现。
 */
#include "status_task.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "my_main.h"

/* 模块实现细节。 */
void Status_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        static bool can1_bus_off_prev;
        static bool can2_bus_off_prev;
        static bool can3_bus_off_prev;
        bool can1_bus_off = ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U);
        bool can2_bus_off = ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0U);
        bool can3_bus_off = ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) != 0U);
        if (can1_bus_off && !can1_bus_off_prev) can1_diag.bus_off_count++;
        if (can2_bus_off && !can2_bus_off_prev) can2_diag.bus_off_count++;
        if (can3_bus_off && !can3_bus_off_prev) can3_diag.bus_off_count++;
        can1_bus_off_prev = can1_bus_off;
        can2_bus_off_prev = can2_bus_off;
        can3_bus_off_prev = can3_bus_off;

       /* 模块实现细节。 */
        BspCan_Service();

        osDelay((osKernelGetTickFreq() * 2U + 999U) / 1000U);
    }
}

