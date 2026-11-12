/*
 * @file pid.h
 * @brief pid模块的用户接口与实现。
 */

#ifndef PID_H
#define PID_H /* 代码说明：相关模块实现。 */

#include <stdbool.h>

typedef struct
{
    float kp; /* 成员变量。 */
    float ki; /* 成员变量。 */
    float kd; /* 成员变量。 */
    float sample_time_ms; /* 成员变量。 */
    float integral; /* 成员变量。 */
    float previous_feedback; /* 成员变量。 */
    float output_limit; /* 成员变量。 */
    float integral_limit; /* 成员变量。 */
    float integral_separation; /* 成员变量。 */
    bool initialized; /* 成员变量。 */
} pid_t;

/* 功能：执行 PID_Init 函数对应的模块操作。 */
bool PID_Init(pid_t *pid, float kp, float ki, float kd, /* 成员变量。 */
              float sample_time_ms, float output_limit, float integral_limit);
/* 误差超过分离阈值时清除积分；阈值为 0 时关闭积分分离。 */
void PID_SetIntegralSeparation(pid_t *pid, float separation);
/* 功能：执行 PID_Reset 函数对应的模块操作。 */
void PID_Reset(pid_t *pid);
/* 功能：执行 PID_Calc 函数对应的模块操作。 */
float PID_Calc(pid_t *pid, float target, float feedback);

#endif
