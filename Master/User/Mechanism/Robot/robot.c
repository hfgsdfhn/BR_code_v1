/*
 * @file robot.c
 * @brief robot模块的用户接口与实现。
 */

#include "robot.h"
#include "bsp_can.h"
#include "upper.h"
#include "arm_task.h"
robot_state_t robot;

/* 功能：执行 Robot_Init 函数对应的模块操作。 */
void Robot_Init(void)
{
    robot = ROBOT_STOP;

}

/* 功能：执行 Robot_Stop 函数对应的模块操作。 */
void Robot_Stop(void)
{
    robot = ROBOT_STOP;
    Arm_TaskStop();
}

/* 功能：执行 BspCan_OnFrame 函数对应的模块操作。 */
void BspCan_OnFrame(const can_frame_t *frame)
{
    Arm_OnFrame(frame);
}
