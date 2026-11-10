/*
 * @file trapezoid_planner.h
 * @brief T 型（梯形）速度规划器：加速度有界，速度对时间呈梯形，不约束加加速度。
 *
 * 与 quintic_planner 的分工：
 *   quintic_planner   位置模式。给起点/终点，一次算出整段轨迹，终点速度必须为 0，
 *                     适合"走完这一段就停住"的关节定位。
 *   trapezoid_planner 速度模式。每个控制周期喂一个目标速度，输出限加速度的速度给定；
 *                     不需要知道终点，可以一直跟下去，速度可以长期保持非零。
 *
 * 曲线形状：
 *   加速段 a = +a_acc，到速后 a = 0，目标下降时 a = -a_dec。
 *   目标恒定且时间足够时就是标准梯形（加速-匀速-减速）；
 *   时间不够自动退化成三角形，不会冲过目标。
 *
 * 用法：
 *   trapezoid_t tp;
 *   Trapezoid_Init(&tp, 1.0f, 1.5f);           // 加速度上限、减速度上限
 *   v = Trapezoid_Update(&tp, v_target, dt);   // 每周期调用，返回值就是速度给定
 *
 * 二维接口用于底盘 vx/vy：按速度矢量模长限幅，而不是逐轴限幅，
 * 这样加速过程中运动方向不会歪。
 */
#ifndef TRAPEZOID_PLANNER_H
#define TRAPEZOID_PLANNER_H

#include <stdint.h>

/* 一维 T 型规划器：单轴速度、轮速、角速度通用。 */
typedef struct
{
    float speed; /* 当前规划速度，输出，可直接当给定用。 */
    float accel; /* 本周期使用的加速度，输出，调试和前馈用。 */
    float a_acc; /* 加速度上限，单位同速度/秒；<= 0 表示旁路规划（仅调试用）。 */
    float a_dec; /* 减速度上限，定义同上；一般取 1.2~1.5 倍 a_acc。 */
} trapezoid_t;

/* 二维 T 型规划器：底盘 vx/vy 共用一个加速度预算。 */
typedef struct
{
    float vx;    /* 当前规划速度 X 分量。 */
    float vy;    /* 当前规划速度 Y 分量。 */
    float a_acc; /* 加速度（矢量模长）上限。 */
    float a_dec; /* 减速度（矢量模长）上限。 */
} trapezoid_2d_t;

/* 初始化一维规划器，速度归零。 */
void Trapezoid_Init(trapezoid_t *tp, float a_acc, float a_dec);

/* 把规划速度直接置为 speed，用于模式切换后从实测速度无缝接上。 */
void Trapezoid_Reset(trapezoid_t *tp, float speed);

/* 单个控制周期调用一次，返回本周期规划速度，同时更新 tp->speed 和 tp->accel。 */
float Trapezoid_Update(trapezoid_t *tp, float target, float dt);

/* 初始化二维规划器，速度归零。 */
void Trapezoid2D_Init(trapezoid_2d_t *tp, float a_acc, float a_dec);

/* 把二维规划速度直接置为 (vx, vy)。 */
void Trapezoid2D_Reset(trapezoid_2d_t *tp, float vx, float vy);

/* 单个控制周期调用一次；vx/vy 输出指针任一可为 NULL。 */
void Trapezoid2D_Update(trapezoid_2d_t *tp, float tx, float ty, float dt, float *vx, float *vy);

/*
 * 制动距离反解：要在 remaining 距离内减速到 0，当前速度上限是 sqrt(2*a_dec*remaining)。
 * 自动跑点到点时把它和 v_max 取小即可；remaining 为负按 0 处理。
 */
float Trapezoid_BrakeSpeedLimit(float remaining, float a_dec);

#endif /* 头文件保护结束。 */
