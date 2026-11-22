#ifndef BR_GRAVITY_H
#define BR_GRAVITY_H

#include <stdbool.h>
#include <stdint.h>

#define BR_ARM_DOF 5U
#define BR_GRAVITY_LINK_COUNT 4U

typedef struct
{
    float origin[3];
    const float (*fixed_rotation)[3];
    float axis[3];
    float mass_kg;
    float com_m[3];
    uint8_t joint_index;
} br_gravity_link_t;

typedef struct
{
    const br_gravity_link_t *link;
    float link_mass_kg[BR_GRAVITY_LINK_COUNT];
    float gravity_base[3];
    float payload_mass_kg;
    float payload_com_m[3];
    const float *tool_offset_m;
    bool calibrated;
} br_gravity_model_t;

extern br_gravity_model_t br_gravity_models[2];

extern volatile float br_link_37_m;
extern volatile float br_link_6_m;
extern volatile float br_link_21_5_m;
extern volatile float br_link_8_m;

extern volatile float br_mass_003_kg;
extern volatile float br_mass_004_kg;
extern volatile float br_mass_005_kg;
extern volatile float br_mass_006_kg;


bool BR_GravityEvaluate(const br_gravity_model_t *model,
                        const float q[BR_ARM_DOF],
                        bool grip,
                        float tau[BR_ARM_DOF],
                        float *potential_j);

bool BR_ForwardKinematics(const br_gravity_model_t *model,
                          const float q[BR_ARM_DOF],
                          float end_world_m[3]);

bool BR_InverseKinematics(const br_gravity_model_t *model,
                          const float target_world_m[3],
                          const float current_q[BR_ARM_DOF],
                          float q_target_deg[BR_ARM_DOF],
                          bool prefer_small_second_rs00);
#endif
