/*
 * @file robot.c
 * @brief robot模块的用户接口与实现。
 */

#include "robot.h"
#include "bsp_can.h"
#include "upper.h"
#include "arm_task.h"
robot_state_t robot;

/* 功能：初始化Robot_Init对应的外设、参数和运行状态。 */
void Robot_Init(void)
{
    robot = ROBOT_STOP;

}

/* 功能：停止Robot_Stop对应的输出并清理运行状态。 */
void Robot_Stop(void)
{
    robot = ROBOT_STOP;
    Arm_TaskStop();
}

/* 功能：接收并解析BspCan_OnFrame对应的输入数据，更新模块状态。 */
void BspCan_OnFrame(const can_frame_t *frame)
{
    Arm_OnFrame(frame);
}
