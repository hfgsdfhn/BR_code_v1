/*
 * @file pid.h
 * @brief pid模块的用户接口与实现。
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>


typedef struct
{
    float kp; /* 成员变量。 */
    float ki; /* 成员变量。 */
    float kd; /* 成员变量。 */
    float integral; /* 成员变量。 */
    float previous_error; /* 成员变量。 */
    float output_limit; /* 成员变量。 */
    float integral_limit; /* 成员变量。 */
    float full_integral_separation; /* 成员变量。 */
    float integral_separation; /* 成员变量。 */
    bool initialized; /* 成员变量。 */
} pid_t;


/* 功能：初始化模块并准备外设资源。 */
bool PID_Init(pid_t *pid                    , float kp            , float ki            , /* 成员变量。 */
              float kd            , float output_limit               , /* 成员变量。 */
              float integral_limit                );


/* 功能：执行 PID_SetIntegralSeparationRange 模块操作并维护相关状态。 */
void PID_SetIntegralSeparationRange(pid_t *pid                   , /* 成员变量。 */
                                    float full_separation                   , /* 成员变量。 */
                                    float separation                   );


/* 功能：执行 PID_Reset 模块操作并维护相关状态。 */
void PID_Reset(pid_t *pid                     );


/* 功能：执行 PID_Calc 模块操作并维护相关状态。 */
float PID_Calc(pid_t *pid                      , float target             , /* 成员变量。 */
               float feedback             , float dt_s                 );

#endif
