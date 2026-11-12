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
    float origin[3], fixed_rotation[3][3], axis[3]; /* 成员变量。 */
    float mass_kg, com_m[3]; /* 成员变量。 */
} br_gravity_link_t;
typedef struct
{
    br_gravity_link_t link[BR_ARM_DOF]; /* 成员变量。 */
    float gravity_base[3]; /* 成员变量。 */
    float payload_mass_kg, payload_com_m[3]; /* 成员变量。 */
    bool calibrated; /* 成员变量。 */
} br_gravity_model_t;

/* 功能：执行 BR_GravityEvaluate 函数对应的模块操作。 */
bool BR_GravityEvaluate(const br_gravity_model_t *model, const float q[BR_ARM_DOF], bool grip, float tau[BR_ARM_DOF], /* 成员变量。 */
                        float *potential_j);
#endif
