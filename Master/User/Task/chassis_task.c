#include "chassis_task.h"
#include "status_task.h"
#include "chassis.h"
#include "board_can.h"
#include "fdcan.h"
send_to_chassis_t chassis_data_;

void Chassis_Task_(void *argument)
{
        uint32_t tick = osKernelGetTickCount();

    const uint32_t period = 2U;  // 2ms，假设系统Tick为1kHz
    for (;;)
    {
        if(osMessageQueueGet(Chassis_QueueHandle,&chassis_data_,0,0) == osOK)
        {
            Chassis_SendSpeed(&hfdcan1,&chassis_data_.target_speed);
        }
        tick += period;
        osDelayUntil(tick);
    }
}