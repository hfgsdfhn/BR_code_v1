




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

            cur_mag = sqrtf(tp->vx * tp->vx + tp->vy * tp->vy);
            tgt_mag = sqrtf(tx * tx + ty * ty);
            limit = (tgt_mag >= cur_mag) ? tp->a_acc : tp->a_dec;

            if (limit > 0.0f)
            {
                step = limit * dt;
                if (dist > step)
                {

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
