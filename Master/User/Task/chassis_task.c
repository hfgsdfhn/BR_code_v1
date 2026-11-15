#include "chassis_task.h"
#include "status_task.h"
#include "chassis.h"
#include "board_can.h"
#include "fdcan.h"
send_to_chassis_t chassis_data_;
float dji_pid[3][3];
uint8_t dji_flag = 0;   //调试用

void Chassis_Task_(void *argument)
{
        uint32_t tick = osKernelGetTickCount();

    const uint32_t period = 2U;  // 2ms，假设系统Tick为1kHz
    for (;;)
    {
        if(osMessageQueueGet(Chassis_QueueHandle,&chassis_data_,0,0) == osOK)
        {
            Chassis_SendSpeed(&hfdcan1,&chassis_data_.target_speed);
            Send_Target_Pos(&hfdcan1, DJI_Board_Target.leg_height_mm);
            if(dji_flag)
            {
                dji_flag = 0;
                SmallBoard_Send_PID(&hfdcan1, 0U, dji_pid[0][0], dji_pid[0][1], dji_pid[0][2]);
                SmallBoard_Send_PID(&hfdcan1, 1U, dji_pid[1][0], dji_pid[1][1], dji_pid[1][2]);
                SmallBoard_Send_PID(&hfdcan1, 2U, dji_pid[2][0], dji_pid[2][1], dji_pid[2][2]);
            }
        }
        tick += period;
        osDelayUntil(tick);
    }
}