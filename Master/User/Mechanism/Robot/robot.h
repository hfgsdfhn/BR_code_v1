/*
 * @file robot.h
 * @brief robot模块的用户接口与实现。
 */

#ifndef MECHANISM_ROBOT_H
#define MECHANISM_ROBOT_H

typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_RUN
} robot_state_t;

extern robot_state_t robot;

/* 功能：初始化Robot_Init对应的外设、参数和运行状态。 */
void Robot_Init(void);

/* 功能：停止Robot_Stop对应的输出并清理运行状态。 */
void Robot_Stop(void);

#endif
