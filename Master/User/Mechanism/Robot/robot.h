/* 整机状态与机构控制接口。 */
#ifndef MECHANISM_ROBOT_H
#define MECHANISM_ROBOT_H

/* ===== 整机状态 ===== */
/*
 * @file robot.h
 * @brief 声明整机状态和控制接口。
 */

typedef enum
{
    ROBOT_STOP = 0, /* 整机停止所有机构输出。 */
    ROBOT_RUN       /* 整机正常执行各机构控制。 */
} robot_state_t;    /* 表示整机控制流程当前的运行状态。 */

extern robot_state_t robot; /* 供各应用模块访问的整机控制状态。 */

/* 功能：初始化整机和所有子系统。 */
void Robot_Init(void);

/* 功能：停止整机全部输出。 */
void Robot_Stop(void);

#endif
