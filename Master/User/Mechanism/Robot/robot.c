/*
 * @file robot.c
 * @brief robot模块的用户接口与实现。
 */

#include "robot.h"
#include "bsp_can.h"
#include "upper.h"
#include "arm_task.h"
robot_state_t robot;

/* 功能：初始化模块并准备外设资源。 */
void Robot_Init(void)
{
    robot = ROBOT_STOP;

}

/* 功能：停止设备输出并清理状态。 */
void Robot_Stop(void)
{
    robot = ROBOT_STOP;
    Arm_TaskStop();
}

/* 功能：接收并解析一帧通信数据。 */
void BspCan_OnFrame(const can_frame_t *frame)
{
    Arm_OnFrame(frame);
}
