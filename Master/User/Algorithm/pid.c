
#include "pid.h"

static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PID_Clamp(float value, float limit)
{
    limit = PID_Abs(limit);

    if (limit == 0.0f)
    {
        return 0.0f;
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

bool PID_Init(pid_t *pid, float kp, float ki, float kd,
              float sample_time_ms, float output_limit, float integral_limit)
{
    if ((pid == 0) || (sample_time_ms <= 0.0f))
    {
        return false;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time_ms = sample_time_ms;
    pid->output_limit = PID_Abs(output_limit);
    pid->integral_limit = PID_Abs(integral_limit);
    pid->integral_separation = 0.0f;
    PID_Reset(pid);
    return true;
}

void PID_SetIntegralSeparation(pid_t *pid, float separation)
{
    if (pid != 0)
    {
        pid->integral_separation = PID_Abs(separation);
    }
}

void PID_Reset(pid_t *pid)
{
    if (pid != 0)
    {
        pid->integral = 0.0f;
        pid->previous_feedback = 0.0f;
        pid->initialized = false;
    }
}

float PID_Calc(pid_t *pid, float target, float feedback)
{
    float error;
    float derivative;
    float output;

    error = target - feedback;
    if ((pid->integral_separation > 0.0f)
        && (PID_Abs(error) > pid->integral_separation))
    {
        pid->integral = 0.0f;
    }

    else
    {
        pid->integral += error;
        pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
    }

    derivative = 0.0f;

    if (pid->initialized)
    {
        derivative = -(feedback - pid->previous_feedback) / pid->sample_time_ms;
    }

    output = pid->kp * error +
             pid->ki * pid->integral +
             pid->kd * derivative;
    pid->previous_feedback = feedback;
    pid->initialized = true;
    return PID_Clamp(output, pid->output_limit);
}
