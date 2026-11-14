/*
 * @file status_task.h
 * @brief status_task模块的用户接口与实现。
 */

#ifndef STATUS_TASK_H
#define STATUS_TASK_H

#include "mins_imu.h"
#include "remote.h"
#include "cmsis_os2.h"
#include "chassis.h"

<<<<<<< HEAD
=======
/* 功能：执行周期任务主流程。 */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
void Status_Task(void *argument);

typedef struct
{
    float dt35_right;
    float dt35_behind;
    remote_t remote;
    chassis_speed_t target_speed;
    imu_t imu;
}send_to_chassis_t;
extern osMessageQueueId_t Chassis_QueueHandle;

void Queue_Init(void);
void Send_to_Chassis(void);


#endif
