/*
 * @file quintic_planner.h
 * @brief 五次多项式位置规划器：给"角度 + 时长"，算出这段运动需要的峰值速度和峰值加速度。
 *
 * 唯一规划入口：
 *   Quintic_Plan(&plan, 起点, 目标, 初速度, 时长);
 *
 * 时长 > 0  —— 时间优先（主用法）：直接采用该时长，正解出
 *                v_peak = 1.875  * |位移| / T
 *                a_peak = 5.7735 * |位移| / T^2
 *              结果写在 plan.peak_speed / plan.peak_accel，供上层检查电机吃不吃得下。
 * 时长 == 0 —— 回退：用 Quintic_SetDefaultLimits() 设的默认限幅反解时长。
 *
 * 曲线族：
 *   s(tau) = k0*tau + (10-6k0)*tau^3 + (8k0-15)*tau^4 + (6-3k0)*tau^5
 * k0 = 0 时精确退化为标准的 10*tau^3 - 15*tau^4 + 6*tau^5。
 * k0 = v0*T/位移 = 起始速度/本段平均速度，电机静止时 v0=0 自动退化为静止起动。
 *
 * 多轴要同时启停时，给所有轴同一个 T 即可，各轴峰值按各自位移等比例缩放。
 */
#ifndef QUINTIC_PLANNER_H
#define QUINTIC_PLANNER_H

#include <stdint.h>

/* 峰值扫描分段数：段数越多限幅越保守。 */
#define QUINTIC_PEAK_SEGMENTS 48U

/* 反解时长时，峰值放大 2% 作为安全余量。 */
#define QUINTIC_PEAK_MARGIN 1.02f

/* 归一化初速度允许范围。k0=2 表示从 2 倍平均速度开始减速停车；
   上限 2.5，再大归一化曲线会出现速度反向。 */
#define QUINTIC_K0_MIN (-2.0f)
#define QUINTIC_K0_MAX (2.5f)

/* 判定"无需运动"的位置容差。 */
#define QUINTIC_EPS 1.0e-6f

typedef struct
{
    float start;      /* 起点位置，单位与调用方一致。 */
    float delta;      /* 总位移 = 目标 - 起点。 */
    float k0;         /* 归一化初速度 = v0*T/位移，已限幅。 */
    float duration;   /* 实际采用时长，单位：秒；0 表示距离为零。 */
    float v_scale;    /* delta/duration，采样时把归一化速度换算成物理速度。 */
    float peak_speed; /* 本段轨迹需要的峰值速度（绝对值）。 */
    float peak_accel; /* 本段轨迹需要的峰值加速度（绝对值）。 */
    uint8_t limited;  /* 1 = 指定时长无法满足默认限幅，已自动延长。 */
} quintic_plan_t;

/* 设定默认限幅：仅在 时长 == 0 时用来反解时长，或在时长优先时做安全校验。
   各电机在自己建立轨迹前调用一次即可。 */
void Quintic_SetDefaultLimits(float max_speed, float max_accel, float min_duration);

/* 唯一规划入口。
   duration > 0：按给定时长规划，并算出该时长下需要的峰值速度/加速度。
   duration == 0：用默认限幅反解时长（静止起动/无时限要求的场合）。 */
void Quintic_Plan(quintic_plan_t *plan, float start, float target, float v0, float duration);

/* 给行程和默认限幅，返回最小可行时长（秒）。多轴同步前先取各轴最大值，
   再作为公共时长下发给每个轴，即可保证同时启停。 */
float Quintic_MinDuration(float delta, float v0);

/* 按已流逝时间采样轨迹，输出位置、速度、加速度（任一输出指针可为 NULL）。 */
void Quintic_Sample(const quintic_plan_t *plan, float elapsed, float *pos, float *vel, float *acc);

/* 变量观察窗口 观察用：最近一次规划的时长、峰值和限幅标志。 */
extern volatile float quintic_last_duration_s;
extern volatile float quintic_last_peak_speed;
extern volatile float quintic_last_peak_accel;
extern volatile uint8_t quintic_last_limited;

#endif /* 头文件保护结束。 */
