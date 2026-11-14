
#include "status_task.h"
#include "bsp_can.h"
#include "my_main.h"

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