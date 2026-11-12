/*
 * @file Chassis_MPC_Ctrl.h
 * @brief Chassis_MPC_Ctrl模块的用户接口与实现。
 */

/**
 * @file
 * @brief 中科大开源的方案调用了qpOASES作为求解器
 *  H = 2(B_TQB_- +R)
 *
 *  g = f =2B_TQ(A_x0- Xref)
 *
 *  A   约束矩阵
 *
 *  lbA、ubA    约束左右边界 lbA <= A*x <= ubA。
 *
 *  lb、ub      优化变量本身的下界和上界    例如对未来控制输入设置幅值限制。
 * @bug     这里约束我只限制了控制量，所以没有速度约束
 *  NV   若优化未来 N 步、每步 nu 个输入，则通常 NV = N * nu。
 *  NC   即 A 有多少行。
 * @version 0.1
 * 未来优化方向：优化运动学模型，引入阻尼
 *              增加可迁移性
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef CHASSIS_MPC_CTRL_H
#define CHASSIS_MPC_CTRL_H

#include "stdint.h"
#include "arm_math.h"
#include "mpc_qpOASES.h"

#define mpc_dt 0.01f    //(s)
#define MPC_MAX_N 50    //预测步数
#define NX   6       // 状态维度
#define NU   3       // 控制维度
#define NV   (MPC_MAX_N*NU)  // 优化变量总维度

typedef struct MPC mpc_t; /* 成员变量。 */

typedef struct {

    float mass;          // 车体质量 (kg)
    float inertia_z;     // 转动惯量 (kg·m²)
    float max_force_x;   // 纵向最大合力 (N)
    float max_force_y;   // 横向最大合力 (N)
    float max_torque_z;  // 最大旋转力矩 (N·m)

} MpcPhysicalParams;

typedef enum {
    x, y, yaw, vx, vy, w
} mpc_state_t;
typedef enum
{
    Fx,Fy,Mz
} mpc_ctrl;
struct MPC
{
    mpc_state_t chassis_state; /* 当前工作模式或状态。 */

    float x[NX]; //状态量
    float u[NU]; //控制量

    float Q[NX]; /* 成员变量。 */
    float R[NU]; /* 成员变量。 */

     uint8_t nWSR;   //最大工作集重计算次数（number of Working Set Recalculations）  限制求解时间

    MpcPhysicalParams RobotParam; /* 成员变量。 */
    uint8_t i;  //运算了多少步
};
/* 功能：执行 MPC_Init 函数对应的模块操作。 */
void MPC_Init(void);
/* 功能：执行 MPC_g_figure 函数对应的模块操作。 */
void MPC_g_figure(const float32_t *x0, const float32_t *X_ref, float32_t *g_data);
/* 功能：执行 MPC_Solve 函数对应的模块操作。 */
void MPC_Solve(const float32_t *x0, const float32_t *X_ref);

/* 代码说明：相关模块实现。 */
const float32_t *MPC_GetH(void);
const float32_t *MPC_GetLb(void);
const float32_t *MPC_GetUb(void);

#endif
