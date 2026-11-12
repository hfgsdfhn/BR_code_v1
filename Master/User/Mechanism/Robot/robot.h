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


/* 功能：执行 Robot_Init 函数对应的模块操作。 */
void Robot_Init(void);


/* 功能：执行 Robot_Stop 函数对应的模块操作。 */
void Robot_Stop(void);

#endif
