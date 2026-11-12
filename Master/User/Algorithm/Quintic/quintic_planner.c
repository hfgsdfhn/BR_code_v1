/*
 * @file quintic_planner.c
 * @brief quintic_planner模块的用户接口与实现。
 */

#include "quintic_planner.h"

#include <math.h>
#include <stddef.h>


static float s_max_speed = 0.0f;
static float s_max_accel = 0.0f;
static float s_min_duration = 0.0f;

volatile float quintic_last_duration_s = 0.0f;
volatile float quintic_last_peak_speed = 0.0f;
volatile float quintic_last_peak_accel = 0.0f;
volatile uint8_t quintic_last_limited = 0U;


/* 功能：执行 Quintic_Shape 函数对应的模块操作。 */
static void Quintic_Shape(float k0, float tau, float *s, float *ds, float *dds)
{
    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;
    float tau5 = tau4 * tau;
    float a3 = 10.0f - 6.0f * k0;
    float a4 = 8.0f * k0 - 15.0f;
    float a5 = 6.0f - 3.0f * k0;

    if (s != NULL)
    {
        *s = k0 * tau + a3 * tau3 + a4 * tau4 + a5 * tau5;
    }
    if (ds != NULL)
    {
        *ds = k0 + 3.0f * a3 * tau2 + 4.0f * a4 * tau3 + 5.0f * a5 * tau4;
    }
    if (dds != NULL)
    {
        *dds = 6.0f * a3 * tau + 12.0f * a4 * tau2 + 20.0f * a5 * tau3;
    }
}


/* 功能：执行 Quintic_Peaks 函数对应的模块操作。 */
static void Quintic_Peaks(float k0, float *peak_ds, float *peak_dds)
{
    float max_ds = 0.0f;
    float max_dds = 0.0f;
    uint32_t i;

    for (i = 0U; i <= QUINTIC_PEAK_SEGMENTS; ++i)
    {
        float tau = (float)i / (float)QUINTIC_PEAK_SEGMENTS;
        float ds;
        float dds;

        Quintic_Shape(k0, tau, NULL, &ds, &dds);
        if (fabsf(ds) > max_ds)
        {
            max_ds = fabsf(ds);
        }
        if (fabsf(dds) > max_dds)
        {
            max_dds = fabsf(dds);
        }
    }
    if (peak_ds != NULL)
    {
        *peak_ds = max_ds;
    }
    if (peak_dds != NULL)
    {
        *peak_dds = max_dds;
    }
}


/* 功能：执行 Quintic_SolveDuration 函数对应的模块操作。 */
static float Quintic_SolveDuration(float delta, float adelta, float v0, float *k0_out)
{
    float t;
    float k0 = 0.0f;
    int i;


    t = 1.875f * adelta / s_max_speed;
    {
        float ta = sqrtf(5.7735027f * adelta / s_max_accel);

        if (ta > t)
        {
            t = ta;
        }
    }
    if (s_min_duration > 0.0f && t < s_min_duration)
    {
        t = s_min_duration;
    }

    if (fabsf(v0) <= QUINTIC_EPS)
    {
        if (k0_out != NULL)
        {
            *k0_out = 0.0f;
        }
        return t;
    }

    for (i = 0; i < 6; ++i)
    {
        float peak_ds;
        float peak_dds;
        float need_v;
        float need_a;
        float t_new;

        k0 = v0 * t / delta;
        if (k0 > QUINTIC_K0_MAX)
        {
            k0 = QUINTIC_K0_MAX;
        }
        if (k0 < QUINTIC_K0_MIN)
        {
            k0 = QUINTIC_K0_MIN;
        }

        Quintic_Peaks(k0, &peak_ds, &peak_dds);
        need_v = adelta * peak_ds * QUINTIC_PEAK_MARGIN / s_max_speed;
        need_a = sqrtf(adelta * peak_dds * QUINTIC_PEAK_MARGIN / s_max_accel);
        t_new = (need_v > need_a) ? need_v : need_a;
        if (s_min_duration > 0.0f && t_new < s_min_duration)
        {
            t_new = s_min_duration;
        }

        if (fabsf(t_new - t) <= 0.0005f * t)
        {
            t = t_new;
            break;
        }
        t = t_new;
    }

    if (k0_out != NULL)
    {
        *k0_out = k0;
    }
    return t;
}


/* 功能：执行 Quintic_Fill 函数对应的模块操作。 */
static void Quintic_Fill(quintic_plan_t *plan, float start, float delta, float k0, float t)
{
    float peak_ds;
    float peak_dds;

    plan->start = start;
    plan->delta = delta;
    plan->k0 = k0;
    plan->duration = t;
    plan->v_scale = delta / t;

    Quintic_Peaks(k0, &peak_ds, &peak_dds);
    plan->peak_speed = fabsf(delta) * peak_ds / t;
    plan->peak_accel = fabsf(delta) * peak_dds / (t * t);

    quintic_last_duration_s = t;
    quintic_last_peak_speed = plan->peak_speed;
    quintic_last_peak_accel = plan->peak_accel;
    quintic_last_limited = plan->limited;
}


/* 功能：执行 Quintic_SetDefaultLimits 函数对应的模块操作。 */
void Quintic_SetDefaultLimits(float max_speed, float max_accel, float min_duration)
{
    s_max_speed = max_speed;
    s_max_accel = max_accel;
    s_min_duration = min_duration;
}


/* 功能：执行 Quintic_MinDuration 函数对应的模块操作。 */
float Quintic_MinDuration(float delta, float v0)
{
    float adelta = fabsf(delta);

    if ((adelta <= QUINTIC_EPS) || !(s_max_speed > 0.0f) || !(s_max_accel > 0.0f) || !isfinite(v0))
    {
        return 0.0f;
    }
/* 功能：执行 Quintic_SolveDuration 函数对应的模块操作。 */
    return Quintic_SolveDuration(delta, adelta, v0, NULL);
}


/* 功能：执行 Quintic_DefaultDuration 函数对应的模块操作。 */
float Quintic_DefaultDuration(float delta, float v0, float requested_duration)
{
    if (requested_duration > QUINTIC_EPS)
    {
        return requested_duration;
    }
/* 功能：执行 Quintic_MinDuration 函数对应的模块操作。 */
    return Quintic_MinDuration(delta, v0);
}


/* 功能：执行 Quintic_Plan 函数对应的模块操作。 */
void Quintic_Plan(quintic_plan_t *plan, float start, float target, float v0, float duration)
{
    float adelta;
    float t;
    float k0 = 0.0f;
    float peak_ds;
    float peak_dds;
    uint8_t has_limits = (uint8_t)((s_max_speed > 0.0f) && (s_max_accel > 0.0f));

    if (plan == NULL)
    {
        return;
    }

    plan->start = start;
    plan->delta = target - start;
    plan->k0 = 0.0f;
    plan->duration = 0.0f;
    plan->v_scale = 0.0f;
    plan->peak_speed = 0.0f;
    plan->peak_accel = 0.0f;
    plan->limited = 0U;

    adelta = fabsf(plan->delta);
    if ((adelta <= QUINTIC_EPS) || !isfinite(v0))
    {
        quintic_last_limited = 0U;
        return;
    }

    if (duration > QUINTIC_EPS)
    {

        t = duration;
        if (s_min_duration > 0.0f && t < s_min_duration)
        {
            t = s_min_duration;
        }

        k0 = v0 * t / plan->delta;
        if (k0 > QUINTIC_K0_MAX)
        {
            k0 = QUINTIC_K0_MAX;
        }
        if (k0 < QUINTIC_K0_MIN)
        {
            k0 = QUINTIC_K0_MIN;
        }

        if (has_limits)
        {
            Quintic_Peaks(k0, &peak_ds, &peak_dds);
            if ((adelta * peak_ds / t > s_max_speed) || (adelta * peak_dds / (t * t) > s_max_accel))
            {

                t = Quintic_SolveDuration(plan->delta, adelta, v0, &k0);
                plan->limited = 1U;
            }
        }
    }
    else if (has_limits)
    {

        t = Quintic_SolveDuration(plan->delta, adelta, v0, &k0);
    }
    else
    {

        quintic_last_limited = 0U;
        return;
    }

    Quintic_Fill(plan, start, plan->delta, k0, t);
}


/* 功能：执行 Quintic_Sample 函数对应的模块操作。 */
void Quintic_Sample(const quintic_plan_t *plan, float elapsed, float *pos, float *vel, float *acc)
{
    float tau;

    if (plan == NULL)
    {
        return;
    }
    if (!(plan->duration > 0.0f))
    {
        if (pos != NULL)
        {
            *pos = plan->start + plan->delta;
        }
        if (vel != NULL)
        {
            *vel = 0.0f;
        }
        if (acc != NULL)
        {
            *acc = 0.0f;
        }
        return;
    }

    tau = elapsed / plan->duration;
    if (tau >= 1.0f)
    {
        if (pos != NULL)
        {
            *pos = plan->start + plan->delta;
        }
        if (vel != NULL)
        {
            *vel = 0.0f;
        }
        if (acc != NULL)
        {
            *acc = 0.0f;
        }
        return;
    }
    if (tau < 0.0f)
    {
        tau = 0.0f;
    }

    {
        float s;
        float ds;
        float dds;

        Quintic_Shape(plan->k0, tau, &s, &ds, &dds);
        if (pos != NULL)
        {
            *pos = plan->start + plan->delta * s;
        }
        if (vel != NULL)
        {
            *vel = plan->v_scale * ds;
        }
        if (acc != NULL)
        {
            *acc = plan->v_scale * dds / plan->duration;
        }
    }
}
