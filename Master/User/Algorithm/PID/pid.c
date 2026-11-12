/*
 * @file pid.c
 * @brief pid模块的用户接口与实现。
 */

#include "pid.h"


/* 功能：执行 pid_clamp 模块操作并维护相关状态。 */
static float pid_clamp(float value, float limit)
{
    if (limit < 0.0f)
    {
        limit = -limit;
    }
    if (limit == 0.0f)
    {
        return value;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}


/* 功能：初始化模块并准备外设资源。 */
bool PID_Init(pid_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit)
{
    if ((pid == 0) || (output_limit < 0.0f) || (integral_limit < 0.0f))
    {
        return false;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->full_integral_separation = 0.0f;
    pid->integral_separation = 0.0f;
    PID_Reset(pid);
    return true;
}


/* 功能：执行 PID_SetIntegralSeparationRange 模块操作并维护相关状态。 */
void PID_SetIntegralSeparationRange(pid_t *pid, float full_separation, float separation)
{
    if (pid == 0)
    {
        return;
    }
    pid->full_integral_separation = (full_separation < 0.0f) ? -full_separation : full_separation;
    pid->integral_separation = (separation < 0.0f) ? -separation : separation;
}


/* 功能：执行 PID_Reset 模块操作并维护相关状态。 */
void PID_Reset(pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = false;
}


/* 功能：执行 PID_Calc 模块操作并维护相关状态。 */
float PID_Calc(pid_t *pid, float target, float feedback, float dt_s)
{
    float error;
    float derivative;
    float previous_integral;
    float integration_weight;
    float absolute_error;
    float candidate_integral;
    float unsaturated_output;
    float output;

    if ((pid == 0) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }
    error = target - feedback;
    previous_integral = pid->integral;
    integration_weight = 1.0f;
    absolute_error = (error < 0.0f) ? -error : error;
    derivative = pid->initialized ? (error - pid->previous_error) / dt_s : 0.0f;

    if ((pid->ki <= 0.0f) || (pid->integral_limit <= 0.0f))
    {
        candidate_integral = 0.0f;
    }
    else if ((pid->integral_separation > 0.0f) && (absolute_error >= pid->integral_separation))
    {
        integration_weight = 0.0f;
        candidate_integral = 0.0f;
    }
    else
    {
        if ((pid->integral_separation > pid->full_integral_separation) &&
            (absolute_error > pid->full_integral_separation))
        {
            integration_weight = (pid->integral_separation - absolute_error) /
                                 (pid->integral_separation - pid->full_integral_separation);
            integration_weight = pid_clamp(integration_weight, 1.0f);
        }
        candidate_integral = pid_clamp(previous_integral + integration_weight * error * dt_s, pid->integral_limit);
    }

    unsaturated_output = pid->kp * error + pid->ki * candidate_integral + pid->kd * derivative;
    output = pid_clamp(unsaturated_output, pid->output_limit);


    if ((integration_weight > 0.0f) && (unsaturated_output != output) && ((error * unsaturated_output) > 0.0f))
    {
        candidate_integral = previous_integral;
        unsaturated_output = pid->kp * error + pid->ki * candidate_integral + pid->kd * derivative;
        output = pid_clamp(unsaturated_output, pid->output_limit);
    }

    pid->integral = candidate_integral;
    pid->previous_error = error;
    pid->initialized = true;
    return output;
}
