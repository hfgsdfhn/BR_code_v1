/*
 * @file status_task.h
 * @brief status_task模块的用户接口与实现。
 */

#ifndef STATUS_TASK_H
#define STATUS_TASK_H

/* 功能：执行Status_Task对应的周期任务并维护运行状态。 */
#include "mins_imu.h"
#include "remote.h"
#include "cmsis_os2.h"
#include "chassis.h"

void Status_Task(void *argument);

extern osMessageQueueId_t Chassis_QueueHandle;

void Queue_Init(void);
void Send_to_Chassis(void);


#endif
