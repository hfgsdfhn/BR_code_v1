/*
 * @file robot.h
 * @brief robot模块的用户接口与实现。
 */

#ifndef MECHANISM_ROBOT_H
#define MECHANISM_ROBOT_H



typedef enum
{
    ROBOT_STOP = 0, /* 成员变量。 */
    ROBOT_RUN /* 成员变量。 */
} robot_state_t;

extern robot_state_t robot; /* 成员变量。 */


/* 功能：初始化模块并准备外设资源。 */
void Robot_Init(void);


/* 功能：停止设备输出并清理状态。 */
void Robot_Stop(void);

#endif
