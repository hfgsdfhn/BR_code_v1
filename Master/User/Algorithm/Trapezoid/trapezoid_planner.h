/*
 * @file trapezoid_planner.h
 * @brief trapezoid_planner模块的用户接口与实现。
 */

#ifndef TRAPEZOID_PLANNER_H
#define TRAPEZOID_PLANNER_H

#include <stdint.h>


typedef struct
{
    float speed; /* 成员变量。 */
    float accel; /* 成员变量。 */
    float a_acc; /* 成员变量。 */
    float a_dec; /* 成员变量。 */
} trapezoid_t;


typedef struct
{
    float vx; /* 成员变量。 */
    float vy; /* 成员变量。 */
    float a_acc; /* 成员变量。 */
    float a_dec; /* 成员变量。 */
} trapezoid_2d_t;


/* 功能：初始化模块并准备外设资源。 */
void Trapezoid_Init(trapezoid_t *tp, float a_acc, float a_dec);


/* 功能：执行 Trapezoid_Reset 模块操作并维护相关状态。 */
void Trapezoid_Reset(trapezoid_t *tp, float speed);


/* 功能：更新控制状态或轨迹数据。 */
float Trapezoid_Update(trapezoid_t *tp, float target, float dt);


/* 功能：初始化模块并准备外设资源。 */
void Trapezoid2D_Init(trapezoid_2d_t *tp, float a_acc, float a_dec);


/* 功能：执行 Trapezoid2D_Reset 模块操作并维护相关状态。 */
void Trapezoid2D_Reset(trapezoid_2d_t *tp, float vx, float vy);


/* 功能：更新控制状态或轨迹数据。 */
void Trapezoid2D_Update(trapezoid_2d_t *tp, float tx, float ty, float dt, float *vx, float *vy);


/* 功能：执行 Trapezoid_BrakeSpeedLimit 模块操作并维护相关状态。 */
float Trapezoid_BrakeSpeedLimit(float remaining, float a_dec);

#endif
