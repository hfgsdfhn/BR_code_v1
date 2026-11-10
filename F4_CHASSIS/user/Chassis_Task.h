/*
 * @file Chassis_Task.h
 * @brief FreeRTOS 底盘控制任务接口。
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "PID.h"
#include "bsp_uart.h"
#include "ak_motor.h"
#include "omni_chassis.h"
#include "path_follow.h"

#define V_MAX 1.5f
#define V_MAX_ERPM (V_MAX * MPS_TO_ERPM)
#define V_STEP 3.5f
#define V_STEP_ERPM (V_STEP * MPS_TO_ERPM)
#define wheel_lf 0
#define wheel_rf 1
#define wheel_lr 2
#define wheel_rr 3
#define world_system 1
#define yaw_correct 1
#define Chassis_Task_T 2

/*---------------------底盘用到的按键---------------------*/
#define shoulder_left  (1U << 0)
#define shoulder_right (1U << 1)
#define key_chassis_stop (1U << 3)
#define left_90        (1U <<4)
#define right_90       (1U <<5)

/* AK60 电机和底盘线速度之间的换算系数。 */
#define ERPM_TO_MPS   0.00008988f
#define MPS_TO_ERPM   (1.0f / ERPM_TO_MPS)

#define DT35_LEFT_PID_LIMIT 2.0f
#define DT35_FRONT_PID_LIMIT 0.5f

#define chassis_to_bolck_x -0.015f
#define chassis_to_bolck_y 0.0f

/* FreeRTOS 底盘控制线程入口。 */
void StartChassisTask(void *argument);

void ImuLink_OnData(const uint8_t *data, uint16_t size);

extern uint8_t dt35_path_state;
extern ak_motor_t ak_wheel[4];
extern path_follow_t follower;

void chassis_init(void);
void chassis_stop(void);
void speed_to_motor(void);
void speed_to_chassis(void);

#endif
