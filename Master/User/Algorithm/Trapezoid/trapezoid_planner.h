/*
 * @file trapezoid_planner.h
 * @brief trapezoid_planner模块的用户接口与实现。
 */

#ifndef TRAPEZOID_PLANNER_H
#define TRAPEZOID_PLANNER_H

#include <stdint.h>

typedef struct
{
    float speed;
    float accel;
    float a_acc;
    float a_dec;
} trapezoid_t;

typedef struct
{
    float vx;
    float vy;
    float a_acc;
    float a_dec;
} trapezoid_2d_t;

/* 功能：初始化Trapezoid_Init对应的外设、参数和运行状态。 */
void Trapezoid_Init(trapezoid_t *tp, float a_acc, float a_dec);

/* 功能：重置或释放Trapezoid_Reset对应的内部状态和资源。 */
void Trapezoid_Reset(trapezoid_t *tp, float speed);

/* 功能：轮询并更新Trapezoid_Update对应的输入和状态。 */
float Trapezoid_Update(trapezoid_t *tp, float target, float dt);

/* 功能：初始化Trapezoid2D_Init对应的外设、参数和运行状态。 */
void Trapezoid2D_Init(trapezoid_2d_t *tp, float a_acc, float a_dec);

/* 功能：重置或释放Trapezoid2D_Reset对应的内部状态和资源。 */
void Trapezoid2D_Reset(trapezoid_2d_t *tp, float vx, float vy);

/* 功能：轮询并更新Trapezoid2D_Update对应的输入和状态。 */
void Trapezoid2D_Update(trapezoid_2d_t *tp, float tx, float ty, float dt, float *vx, float *vy);

/* 功能：处理Trapezoid_BrakeSpeedLimit对应的输入并更新相关状态。 */
float Trapezoid_BrakeSpeedLimit(float remaining, float a_dec);

#endif
