#include "status_task.h"
#include "bsp_can.h"
#include "mins_imu.h"
#include "my_main.h"

osMessageQueueId_t Chassis_QueueHandle;

void Queue_Init(void)                                                                                            //sb刘天赫
{                                                                                            //sb刘天赫
    Chassis_QueueHandle = osMessageQueueNew(1U, sizeof(send_to_chassis_t), NULL);                                                                                            //sb刘天赫
}                                                                                            //sb刘天赫
                                                                                            //sb刘天赫
void Send_to_Chassis(void)                                                                                            //sb刘天赫
{                                                                                            //sb刘天赫
    static chassis_speed_t planned_speed = {0.0f, 0.0f, 0.0f};                                                                                            //sb刘天赫
    static float target_yaw;                                                                                            //sb刘天赫
    send_to_chassis_t chassis_data = {0};                                                                                            //sb刘天赫
    Remote_GetState(&chassis_data.remote);                                                                                            //sb刘天赫
    chassis_data.target_speed = planned_speed;                                                                                            //sb刘天赫
    chassis_data.target_yaw = target_yaw;                                                                                            //sb刘天赫
    chassis_data.current_yaw = g_imu.euler.yaw;                                                                                            //sb刘天赫
    key_to_chassis(&chassis_data);                                                                                            //sb刘天赫
    joy_to_mps(&chassis_data);                                                                                            //sb刘天赫
    planned_speed = chassis_data.target_speed;                                                                                            //sb刘天赫
    target_yaw = chassis_data.target_yaw;                                                                                            //sb刘天赫
    osMessageQueuePut(Chassis_QueueHandle, &chassis_data, 0U, 0U);                                                                                            //sb刘天赫
}                                                                                            //sb刘天赫
                                                                                            //sb刘天赫
void Status_Task(void *argument)                                                                                            //sb刘天赫
{                                                                                            //sb刘天赫
    (void)argument;                                                                                            //sb刘天赫
    uint32_t tick = osKernelGetTickCount();                                                                                            //sb刘天赫
    static bool can1_bus_off_prev;                                                                                            //sb刘天赫
    static bool can2_bus_off_prev;                                                                                            //sb刘天赫
    static bool can3_bus_off_prev;                                                                                            //sb刘天赫
                                                                                            //sb刘天赫
    for (;;)                                                                                            //sb刘天赫
    {                                                                                            //sb刘天赫
        bool can1_bus_off = ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U);                                                                                            //sb刘天赫
        bool can2_bus_off = ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0U);                                                                                            //sb刘天赫
        bool can3_bus_off = ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) != 0U);                                                                                            //sb刘天赫
        if (can1_bus_off && !can1_bus_off_prev) can1_diag.bus_off_count++;                                                                                            //sb刘天赫
        if (can2_bus_off && !can2_bus_off_prev) can2_diag.bus_off_count++;                                                                                            //sb刘天赫
        if (can3_bus_off && !can3_bus_off_prev) can3_diag.bus_off_count++;                                                                                            //sb刘天赫
        can1_bus_off_prev = can1_bus_off;                                                                                            //sb刘天赫
        can2_bus_off_prev = can2_bus_off;
        can3_bus_off_prev = can3_bus_off;
        BspCan_Service();
        Send_to_Chassis();
        tick += 2U;
        osDelayUntil(tick);
    }
}
