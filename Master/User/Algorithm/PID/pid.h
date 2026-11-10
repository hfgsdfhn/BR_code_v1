#ifndef PID_H
#define PID_H /* PID 算法头文件保护宏。 */

#include <stdbool.h>

/* PID 控制器参数、状态以及积分分离和输出限幅配置。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_limit;
    float integral_limit;
    float full_integral_separation;
    float integral_separation;
    bool initialized;
} pid_t;

/* 初始化 PID 参数、积分分离参数，并清零运行状态。 */
bool PID_Init(pid_t *pid /* 待初始化的 PID 对象。 */, float kp /* 比例增益。 */, float ki /* 积分增益。 */,
              float kd /* 微分增益。 */, float output_limit /* 输出绝对值限幅。 */,
              float integral_limit /* 积分项绝对值限幅。 */);

/* 设置积分完全分离和渐进分离的误差范围。 */
void PID_SetIntegralSeparationRange(pid_t *pid /* 待配置的 PID 对象。 */,
                                    float full_separation /* 完全关闭积分的误差阈值。 */,
                                    float separation /* 逐步恢复积分的误差阈值。 */);

/* 清零积分、历史误差和首次微分计算状态。 */
void PID_Reset(pid_t *pid /* 待清零状态的 PID 对象。 */);

/* 计算带积分分离和抗饱和处理的 PID 输出。 */
float PID_Calc(pid_t *pid /* 参与本次计算的 PID 对象。 */, float target /* 当前目标值。 */,
               float feedback /* 当前反馈值。 */, float dt_s /* 采样周期，单位：秒。 */);

#endif
