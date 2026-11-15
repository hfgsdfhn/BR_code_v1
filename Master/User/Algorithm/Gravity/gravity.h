/*
 * @file gravity.h
 * @brief gravity模块的用户接口与实现。
 */

#ifndef BR_GRAVITY_H
#define BR_GRAVITY_H
#include <stdbool.h>
#define BR_ARM_DOF 5U

typedef struct
{
    float origin[3], fixed_rotation[3][3], axis[3];
    float mass_kg, com_m[3];
} br_gravity_link_t;
typedef struct
{
    br_gravity_link_t link[BR_ARM_DOF];
    float gravity_base[3];
    float payload_mass_kg, payload_com_m[3];
    bool calibrated;
} br_gravity_model_t;

/* 功能：计算BR_GravityEvaluate对应的控制量或规划结果。 */
bool BR_GravityEvaluate(const br_gravity_model_t *model, const float q[BR_ARM_DOF], bool grip, float tau[BR_ARM_DOF],
                        float *potential_j);
#endif
