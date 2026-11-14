#include "status_task.h"
#include "bsp_can.h"
#include "my_main.h"

/* 功能：执行Status_Task对应的周期任务并维护运行状态。 */
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

        BspCan_Service();

        osDelay((osKernelGetTickFreq() * 2U + 999U) / 1000U);
    }
}
osMessageQueueId_t Chassis_QueueHandle;

void Queue_Init(void)
{
    Chassis_QueueHandle = osMessageQueueNew(1,sizeof(send_to_chassis_t),NULL);
}

void Send_to_Chassis()
{
    static chassis_speed_t planned_speed = {0.0f, 0.0f, 0.0f};
    send_to_chassis_t chassis_data = {0};
    Remote_GetState(&chassis_data.remote);
    joy_to_mps(&chassis_data.remote, &planned_speed);
    chassis_data.target_speed = planned_speed;

    osMessageQueuePut(Chassis_QueueHandle, &chassis_data, 0U, 0U);
}

void Status_Task(void *argument)
{

    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
        Send_to_Chassis();

        /* 2ms 周期调度 */
        tick += 2U;
        osDelayUntil(tick);
    }
}
