/*
 * @file trapezoid_planner.c
 * @brief T 型速度规划器的实现；纯算法，不依赖任何硬件。
 */

#include "trapezoid_planner.h"

#include <math.h>
#include <stddef.h>

void Trapezoid_Init(trapezoid_t *tp, float a_acc, float a_dec)
{
    if (tp == NULL)
    {
        return;
    }

    tp->speed = 0.0f;
    tp->accel = 0.0f;
    tp->a_acc = (a_acc > 0.0f) ? a_acc : 0.0f;
    tp->a_dec = (a_dec > 0.0f) ? a_dec : 0.0f;
}

void Trapezoid_Reset(trapezoid_t *tp, float speed)
{
    if (tp == NULL)
    {
        return;
    }

    tp->speed = speed;
    tp->accel = 0.0f;
}

float Trapezoid_Update(trapezoid_t *tp, float target, float dt)
{
    float dv;
    float limit;
    float step;

    if (tp == NULL)
    {
        return 0.0f;
    }

    /* 目标异常时保持当前速度，不让 NaN 顺着规划器灌进电流环。 */
    if (!isfinite(target))
    {
        return tp->speed;
    }

    if (!(dt > 0.0f))
    {
        return tp->speed;
    }

    dv = target - tp->speed;
    limit = (dv >= 0.0f) ? tp->a_acc : tp->a_dec;
    if (limit <= 0.0f)
    {
        /* 未配置限幅：直通，便于对比"有规划/无规划"两种行为。 */
        tp->speed = target;
        tp->accel = 0.0f;
        return tp->speed;
    }

    step = limit * dt;
    if (dv > step)
    {
        tp->accel = limit;
        tp->speed += step;
    }
    else if (dv < -step)
    {
        tp->accel = -limit;
        tp->speed -= step;
    }
    else
    {
        /* 本周期就能到速：收尾，加速度取实际需要值（必然不超过限幅）。 */
        tp->accel = dv / dt;
        tp->speed = target;
    }

    return tp->speed;
}

void Trapezoid2D_Init(trapezoid_2d_t *tp, float a_acc, float a_dec)
{
    if (tp == NULL)
    {
        return;
    }

    tp->vx = 0.0f;
    tp->vy = 0.0f;
    tp->a_acc = (a_acc > 0.0f) ? a_acc : 0.0f;
    tp->a_dec = (a_dec > 0.0f) ? a_dec : 0.0f;
}

void Trapezoid2D_Reset(trapezoid_2d_t *tp, float vx, float vy)
{
    if (tp == NULL)
    {
        return;
    }

    tp->vx = vx;
    tp->vy = vy;
}

void Trapezoid2D_Update(trapezoid_2d_t *tp, float tx, float ty, float dt, float *vx, float *vy)
{
    float dx;
    float dy;
    float dist;
    float cur_mag;
    float tgt_mag;
    float limit;
    float step;
    float scale;

    if (tp == NULL)
    {
        return;
    }

    /* 目标异常时原地保持，避免 NaN 传播。 */
    if (!isfinite(tx) || !isfinite(ty))
    {
        tx = tp->vx;
        ty = tp->vy;
    }

    if (dt > 0.0f)
    {
        dx = tx - tp->vx;
        dy = ty - tp->vy;
        dist = sqrtf(dx * dx + dy * dy);

        if (dist > 0.0f)
        {
            /* 目标模长不小于当前模长算加速，否则算减速。 */
            cur_mag = sqrtf(tp->vx * tp->vx + tp->vy * tp->vy);
            tgt_mag = sqrtf(tx * tx + ty * ty);
            limit = (tgt_mag >= cur_mag) ? tp->a_acc : tp->a_dec;

            if (limit > 0.0f)
            {
                step = limit * dt;
                if (dist > step)
                {
                    /* 只限制速度矢量的变化量，方向保持不变。 */
                    scale = step / dist;
                    tp->vx += dx * scale;
                    tp->vy += dy * scale;
                }
                else
                {
                    tp->vx = tx;
                    tp->vy = ty;
                }
            }
            else
            {
                tp->vx = tx;
                tp->vy = ty;
            }
        }
    }

    if (vx != NULL)
    {
        *vx = tp->vx;
    }
    if (vy != NULL)
    {
        *vy = tp->vy;
    }
}

float Trapezoid_BrakeSpeedLimit(float remaining, float a_dec)
{
    if (!(remaining > 0.0f) || !(a_dec > 0.0f))
    {
        return 0.0f;
    }

    return sqrtf(2.0f * a_dec * remaining);
}
