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
    if (frame == NULL)
        return;

    /* Keep the bus origin explicit.  Motor *_OnFrame() handlers also verify
     * frame->hfdcan against each motor's bound CAN peripheral, so equal IDs
     * seen on another bus cannot update the wrong motor state. */
    if (frame->hfdcan == &hfdcan2)
    {
        /* CAN2 feedback is dispatched through the CAN3-side mechanism path. */
        Arm_OnFrame(frame);
    }
    else if (frame->hfdcan == &hfdcan3)
    {
        /* CAN3 feedback is dispatched through the CAN2-side mechanism path. */
        Arm_OnFrame(frame);
    }
    else if (frame->hfdcan == &hfdcan1)
    {
        /* CAN1 frames are parsed as CAN1 traffic; motor source checks reject
         * same-ID CAN1 frames unless a motor is explicitly bound to CAN1. */
        Arm_OnFrame(frame);
    }
}
