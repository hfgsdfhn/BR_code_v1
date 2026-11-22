#include "chassis_task.h"
#include "status_task.h"
#include "chassis.h"
#include "board_can.h"
#include "fdcan.h"
send_to_chassis_t chassis_data_;
uint8_t dji_flag = 1;   //调试用
DJI_leg_pos_e dji_leg_pos_type = 0;
static float all_leg_pos;
static float front_leg_pos;
static float behind_leg_pos;

float leg_target_pos_a = 0;
float leg_target_pos_f = 0;
float leg_target_pos_b = 0;

void Chassis_Task_(void *argument)
{
    uint32_t tick = osKernelGetTickCount();
    const uint32_t period = 2U;
    (void)argument;
    for (;;)
    {
        send_to_chassis_t status;
        if (osMessageQueueGet(Chassis_QueueHandle, &status, NULL, 0U) == osOK)
        {
            if(status.chassis_emmergency_stop ==0)
            Chassis_SendSpeed(&hfdcan1, &status.target_speed);
            else {
                chassis_speed_t stop = { .vx = 0.0f, .vy = 0.0f, .vw = 0.0f };
                Chassis_SendSpeed(&hfdcan1, &stop);
            }

            leg_task();

            if(dji_flag)
            {
                dji_flag = 0U;
                SmallBoard_Send_PID(&hfdcan1, 0U, DJI_Board_Target.PID_K[0][0], DJI_Board_Target.PID_K[0][1], DJI_Board_Target.PID_K[0][2]);
                SmallBoard_Send_PID(&hfdcan1, 1U, DJI_Board_Target.PID_K[1][0], DJI_Board_Target.PID_K[1][1], DJI_Board_Target.PID_K[1][2]);
                SmallBoard_Send_PID(&hfdcan1, 2U, DJI_Board_Target.PID_K[2][0], DJI_Board_Target.PID_K[2][1], DJI_Board_Target.PID_K[2][2]);
            } 
        }
        tick += period;
        osDelayUntil(tick);
    }
}
