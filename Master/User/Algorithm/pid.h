/*
 * @file pid.h
 * @brief pid模块的用户接口与实现。
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float sample_time_ms;
    float integral;
    float previous_feedback;
    float output_limit;
    float integral_limit;
    float integral_separation;
    bool initialized;
} pid_t;

/* 功能：初始化PID_Init对应的外设、参数和运行状态。 */
bool PID_Init(pid_t *pid, float kp, float ki, float kd,
              float sample_time_ms, float output_limit, float integral_limit);
/* 误差超过分离阈值时清除积分；阈值为 0 时关闭积分分离。 */
void PID_SetIntegralSeparation(pid_t *pid, float separation);
/* 功能：重置或释放PID_Reset对应的内部状态和资源。 */
void PID_Reset(pid_t *pid);
/* 功能：计算PID_Calc对应的控制量或规划结果。 */
float PID_Calc(pid_t *pid, float target, float feedback);

#endif
