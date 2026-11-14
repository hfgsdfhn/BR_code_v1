/*
 * @file gravity.c
 * @brief gravity模块的用户接口与实现。
 */

#include "gravity.h"
#include <math.h>
#include <string.h>

/* 功能：执行 mv 模块操作并维护相关状态。 */
static void mv(const float r[3][3], const float v[3], float out[3])
{
    unsigned i;
    for (i = 0; i < 3; i++)
        out[i] = r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2];
}

/* 功能：执行 mm 模块操作并维护相关状态。 */
static void mm(const float a[3][3], const float b[3][3], float out[3][3])
{
    unsigned i, j, k;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            out[i][j] = 0;
            for (k = 0; k < 3; k++)
                out[i][j] += a[i][k] * b[k][j];
        }
}

/* 功能：执行 vec_ok 模块操作并维护相关状态。 */
static bool vec_ok(const float v[3])
{
/* 模块实现细节。 */
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/* 功能：执行 rotation_ok 模块操作并维护相关状态。 */
static bool rotation_ok(const float r[3][3])
{
    unsigned i, j, k;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            float dot = 0;
            for (k = 0; k < 3; k++)
                dot += r[k][i] * r[k][j];
            if (!isfinite(dot) || fabsf(dot - (i == j ? 1.0f : 0.0f)) > 0.001f)
                return false;
        }
    return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
               r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]) >
           0.999f;
}

/* 功能：执行 add_mass 模块操作并维护相关状态。 */
static void add_mass(float mass, const float c[3], unsigned last, const float origins[BR_ARM_DOF][3],
                     const float axes[BR_ARM_DOF][3], const float g[3], float tau[BR_ARM_DOF], float *u)
{
    unsigned j;
    *u -= mass * (g[0] * c[0] + g[1] * c[1] + g[2] * c[2]);
    for (j = 0; j <= last; j++)
    {
        float d[3] = {c[0] - origins[j][0], c[1] - origins[j][1], c[2] - origins[j][2]};
        const float *a = axes[j];

        tau[j] -= mass * (g[0] * (a[1] * d[2] - a[2] * d[1]) + g[1] * (a[2] * d[0] - a[0] * d[2]) +
                          g[2] * (a[0] * d[1] - a[1] * d[0]));
    }
}

/* 功能：执行 BR_GravityEvaluate 模块操作并维护相关状态。 */
bool BR_GravityEvaluate(const br_gravity_model_t *m, const float q[BR_ARM_DOF], bool grip, float tau[BR_ARM_DOF],
                        float *potential)
{
    float r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, p[3] = {0}, origins[BR_ARM_DOF][3], axes[BR_ARM_DOF][3];
    float result[BR_ARM_DOF] = {0}, u = 0;
    unsigned n, i, j;
    if (!tau)
        return false;
    memset(tau, 0, BR_ARM_DOF * sizeof(float));
    if (potential)
        *potential = 0;
    if (!m || !q || !m->calibrated || !vec_ok(m->gravity_base) || !isfinite(m->payload_mass_kg) ||
        m->payload_mass_kg < 0 || !vec_ok(m->payload_com_m))
        return false;
    for (n = 0; n < BR_ARM_DOF; n++)
    {
        const br_gravity_link_t *l = &m->link[n];
        float base[3][3], rot[3][3], next[3][3], d[3], c[3], s, co, t;
        const float *a = l->axis;
        if (!isfinite(q[n]) || !vec_ok(l->origin) || !vec_ok(a) || !vec_ok(l->com_m) ||
            !rotation_ok(l->fixed_rotation) || !isfinite(l->mass_kg) || l->mass_kg < 0 ||
            fabsf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] - 1) > 0.001f)
            return false;
        mv(r, l->origin, d);
        for (i = 0; i < 3; i++)
            origins[n][i] = p[i] += d[i];
        mm(r, l->fixed_rotation, base);
        mv(base, a, axes[n]);
        s = sinf(q[n]);
        co = cosf(q[n]);
        t = 1 - co;
        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
                rot[i][j] = t * a[i] * a[j] + (i == j ? co : 0);
        rot[0][1] -= s * a[2];
        rot[0][2] += s * a[1];
        rot[1][0] += s * a[2];
        rot[1][2] -= s * a[0];
        rot[2][0] -= s * a[1];
        rot[2][1] += s * a[0];
        mm(base, rot, next);
        memcpy(r, next, sizeof(r));
        mv(r, l->com_m, c);
        for (i = 0; i < 3; i++)
            c[i] += p[i];
        add_mass(l->mass_kg, c, n, origins, axes, m->gravity_base, result, &u);
    }
    if (grip)
    {
        float c[3];
        mv(r, m->payload_com_m, c);
        for (i = 0; i < 3; i++)
            c[i] += p[i];
        add_mass(m->payload_mass_kg, c, BR_ARM_DOF - 1, origins, axes, m->gravity_base, result, &u);
    }
    if (!isfinite(u))
        return false;
    for (i = 0; i < BR_ARM_DOF; i++)
        if (!isfinite(result[i]))
            return false;
    memcpy(tau, result, sizeof(result));
    if (potential)
        *potential = u;
    return true;
}
