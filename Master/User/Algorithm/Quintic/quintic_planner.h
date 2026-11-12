/*
 * @file quintic_planner.h
 * @brief quintic_planner模块的用户接口与实现。
 */

#ifndef QUINTIC_PLANNER_H
#define QUINTIC_PLANNER_H

#include <stdint.h>


#define QUINTIC_PEAK_SEGMENTS 48U


#define QUINTIC_PEAK_MARGIN 1.02f


#define QUINTIC_K0_MIN (-2.0f)
#define QUINTIC_K0_MAX (2.5f)


#define QUINTIC_EPS 1.0e-6f

typedef struct
{
    float start; /* 成员变量。 */
    float delta; /* 成员变量。 */
    float k0; /* 成员变量。 */
    float duration; /* 成员变量。 */
    float v_scale; /* 成员变量。 */
    float peak_speed; /* 成员变量。 */
    float peak_accel; /* 成员变量。 */
    uint8_t limited; /* 成员变量。 */
} quintic_plan_t;


/* 功能：执行 Quintic_SetDefaultLimits 模块操作并维护相关状态。 */
void Quintic_SetDefaultLimits(float max_speed, float max_accel, float min_duration);


/* 功能：规划设备运动轨迹。 */
void Quintic_Plan(quintic_plan_t *plan, float start, float target, float v0, float duration);


/* 模块实现细节。 */
float Quintic_MinDuration(float delta, float v0);

/* 模块实现细节。 */
float Quintic_DefaultDuration(float delta, float v0, float requested_duration);


/* 功能：执行 Quintic_Sample 模块操作并维护相关状态。 */
void Quintic_Sample(const quintic_plan_t *plan, float elapsed, float *pos, float *vel, float *acc);


extern volatile float quintic_last_duration_s; /* 成员变量。 */
extern volatile float quintic_last_peak_speed; /* 成员变量。 */
extern volatile float quintic_last_peak_accel; /* 成员变量。 */
extern volatile uint8_t quintic_last_limited; /* 成员变量。 */

#endif
