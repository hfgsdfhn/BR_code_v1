#include "chassis_task.h"
#include "status_task.h"
#include "chassis.h"
#include "path_follow.h"
#include "math.h"
#include "board_can.h"
#include "mins_imu.h"
#include "fdcan.h"
send_to_chassis_t chassis_data_;
uint8_t dji_flag = 1;   //调试用
DJI_leg_pos_e dji_leg_pos_type = 0;
void World_Change(send_to_chassis_t *send);
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
            if(PC3) PathFollow_Calc(&path_to_xiepo,status.chassis_walk.x,status.chassis_walk.y,&status.target_speed);
            if(status.chassis_emmergency_stop == 0)
            {
                World_Change(&status);
                if (PA0 && dt35_data.left_distance/1000.0f <= 0.5f)    status.target_speed.vy = -PID_Calc(&dt35_pid, 0.2f, dt35_data.left_distance/1000.0f,0.002f);
                Chassis_SendSpeed(&hfdcan1, &status.target_speed);
            }
            else 
            {
                chassis_speed_t stop = { .vx = 0.0f, .vy = 0.0f, .vw = 0.0f };
                Chassis_SendSpeed(&hfdcan1, &stop);
            }

            leg_task();
            
            // if(dji_flag)
            // {
            //     dji_flag = 0U;
            //     SmallBoard_Send_PID(&hfdcan1, 0U, DJI_Board_Target.PID_K[0][0], DJI_Board_Target.PID_K[0][1], DJI_Board_Target.PID_K[0][2]);
            //     SmallBoard_Send_PID(&hfdcan1, 1U, DJI_Board_Target.PID_K[1][0], DJI_Board_Target.PID_K[1][1], DJI_Board_Target.PID_K[1][2]);
            //     SmallBoard_Send_PID(&hfdcan1, 2U, DJI_Board_Target.PID_K[2][0], DJI_Board_Target.PID_K[2][1], DJI_Board_Target.PID_K[2][2]);
            // } 
        }
        tick += period;
        osDelayUntil(tick);
    }
}

void World_Change(send_to_chassis_t *send)
{
    float chassis_vx = send->target_speed.vx;
    float chassis_vy = send->target_speed.vy;
    float sint = sinf(send->current_yaw);
    float cost = cosf(send->current_yaw);
    send->target_speed.vx = chassis_vx * cost - chassis_vy *sint;
    send->target_speed.vy = chassis_vx * sint + chassis_vy *cost;
}