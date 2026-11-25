#include "gravity.h"

#include <math.h>
#include <string.h>

#define BR_PI 3.14159265358979323846f
#define BR_DEG_TO_RAD (BR_PI / 180.0f)

volatile float br_link_37_m = 0.220000f;
volatile float br_link_6_m = 0.058000f;
volatile float br_link_21_5_m = 0.215000f;
volatile float br_link_8_m  = 0.120000f;
static const float br_command_zero_ground_deg[4] = {15.0f, 150.0f, 195.0f, 212.0f};

volatile float br_mass_003_kg = 0.770000000000f;
volatile float br_mass_004_kg = 0.172000000000f;
volatile float br_mass_005_kg = 0.530000000000f;
volatile float br_mass_006_kg = 1.108000000000f;
static const float br_com_37_m = 0.21244607f;
static const float br_com_6_m = 0.049223881f;
static const float br_com_21_5_m = 0.049902387f;
static const float br_com_8_m = 0.080391522f;

static const float br_tool_offset_m[3] = {0.0f, 0.0f, 0.0f};
static const float br_identity[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

static br_gravity_link_t br_g_links[BR_GRAVITY_LINK_COUNT] =
{
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = 0.770000000000f, .com_m = {br_com_37_m, 0.0f, 0.0f}, .joint_index = 1U},
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = 0.172000000000f, .com_m = {br_com_6_m, 0.0f, 0.0f}, .joint_index = 2U},
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = 0.530000000000f, .com_m = {br_com_21_5_m, 0.0f, 0.0f}, .joint_index = 2U},
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = 1.108000000000f, .com_m = {br_com_8_m, 0.0f, 0.0f}, .joint_index = 3U}
};

br_gravity_model_t br_gravity_models[2] =
{
    {.link = br_g_links, .link_mass_kg = {0.770000000000f, 0.172000000000f, 0.530000000000f, 1.108000000000f}, .gravity_base = {0.0f, -9.80665f, 0.0f},
     .payload_mass_kg = 0.174f, .payload_com_m = {0.0f, 0.0f, 0.0f},
     .tool_offset_m = br_tool_offset_m, .calibrated = true},
    {.link = br_g_links, .link_mass_kg = {0.770000000000f, 0.172000000000f, 0.530000000000f, 1.108000000000f}, .gravity_base = {0.0f, -9.80665f, 0.0f},
     .payload_mass_kg = 0.174f, .payload_com_m = {0.0f, 0.0f, 0.0f},
     .tool_offset_m = br_tool_offset_m, .calibrated = true}
};

typedef struct { float x; float y; } br_vec2_t;

static br_vec2_t br_add(br_vec2_t a, br_vec2_t b)
{ return (br_vec2_t){a.x + b.x, a.y + b.y}; }

static br_vec2_t br_polar(float length, float angle)
{ return (br_vec2_t){length * cosf(angle), length * sinf(angle)}; }



static void br_pose(const float q[BR_ARM_DOF], br_vec2_t joint[4], br_vec2_t com[4], br_vec2_t *tool)
{
    const float a1 = br_command_zero_ground_deg[0] * BR_DEG_TO_RAD + q[1];
    const float a2 = br_command_zero_ground_deg[1] * BR_DEG_TO_RAD + q[1] - q[2];
    const float a3 = br_command_zero_ground_deg[2] * BR_DEG_TO_RAD + q[1] - q[2];
    const float a4 = br_command_zero_ground_deg[3] * BR_DEG_TO_RAD + q[1] - q[2] - q[3];
    const br_vec2_t p0 = {0.0f, 0.0f};
    const br_vec2_t p1 = br_add(p0, br_polar(br_link_37_m, a1));
    const br_vec2_t p2 = br_add(p1, br_polar(br_link_6_m, a2));
    const br_vec2_t p3 = br_add(p2, br_polar(br_link_21_5_m, a3));
    const br_vec2_t p4 = br_add(p3, br_polar(br_link_8_m, a4));

    joint[0] = p0; joint[1] = p1; joint[2] = p2; joint[3] = p3;
    com[0] = br_add(p0, br_polar(br_com_37_m, a1));
    com[1] = br_add(p1, br_polar(br_com_6_m, a2));
    com[2] = br_add(p2, br_polar(br_com_21_5_m, a3));
    com[3] = br_add(p3, br_polar(br_com_8_m, a4));
    *tool = p4;
}


static float br_torque(br_vec2_t origin, br_vec2_t center, float mass, const float gravity[3])
{
    const float rx = center.x - origin.x;
    const float ry = center.y - origin.y;
    return -mass * (rx * gravity[1] - ry * gravity[0]);
}

bool BR_ForwardKinematics(const br_gravity_model_t *model,
                          const float q[BR_ARM_DOF], float end_world_m[3])
{
    const float l1 = 0.365000f;
    const float l2 = 0.058000f;
    const float l3 = 0.215000f;
    const float l4 = 0.120000f;
    const float a1_zero = 15.0f * BR_DEG_TO_RAD;
    const float a2_zero = 150.0f * BR_DEG_TO_RAD;
    const float a3_zero = 195.0f * BR_DEG_TO_RAD;
    const float a4_zero = 212.0f * BR_DEG_TO_RAD;
    float a1, a2, a3, a4, planar_x, planar_y;

    if (model == NULL || q == NULL || end_world_m == NULL || model->link == NULL)
        return false;
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        if (!isfinite(q[i])) return false;

    a1 = a1_zero + q[1];
    a2 = a2_zero + q[1] - q[2];
    a3 = a3_zero + q[1] - q[2];
    a4 = a4_zero + q[1] - q[2] - q[3];
    planar_x = l1 * cosf(a1) + l2 * cosf(a2) + l3 * cosf(a3) + l4 * cosf(a4);
    planar_y = l1 * sinf(a1) + l2 * sinf(a2) + l3 * sinf(a3) + l4 * sinf(a4);

    if (model == &br_gravity_models[1])
        end_world_m[0] = planar_x * sinf(q[0]);
    else
        end_world_m[0] = -planar_x * sinf(q[0]);
    end_world_m[1] = -planar_x * cosf(q[0]);
    end_world_m[2] = planar_y;
    return true;
}

static float br_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

bool BR_InverseKinematics(const br_gravity_model_t *model,
                          const float target_world_m[3],
                          const float current_q[BR_ARM_DOF],
                          float q_target_deg[BR_ARM_DOF],
                          bool prefer_small_second_rs00)
{
    const float l1 = 0.365000f;
    const float l2 = 0.058000f;
    const float l3 = 0.215000f;
    const float l4 = 0.120000f;
    const float a1_zero = 15.0f * BR_DEG_TO_RAD;
    const float a2_zero = 150.0f * BR_DEG_TO_RAD;
    const float a3_zero = 195.0f * BR_DEG_TO_RAD;
    const float a4_zero = 212.0f * BR_DEG_TO_RAD;
    const float pi = BR_PI;
    const float welded_x = l2 * cosf(a2_zero) + l3 * cosf(a3_zero);
    const float welded_y = l2 * sinf(a2_zero) + l3 * sinf(a3_zero);
    const float welded_length = sqrtf(welded_x * welded_x + welded_y * welded_y);
    const float welded_zero = atan2f(welded_y, welded_x);
    float radius, yaw, a4, wrist_x, wrist_y, c2, s2_abs;
    float candidate[2][BR_ARM_DOF];
    unsigned best = 0U;

    if (model == NULL || target_world_m == NULL || current_q == NULL ||
        q_target_deg == NULL || model->link == NULL)
        return false;
    for (unsigned i = 0U; i < 3U; ++i)
        if (!isfinite(target_world_m[i])) return false;
    if (target_world_m[1] < 0.0f) return false;
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        if (!isfinite(current_q[i])) return false;

    radius = sqrtf(target_world_m[0] * target_world_m[0] +
                   target_world_m[1] * target_world_m[1]);
    if (radius > 1.0e-6f)
    {
        yaw = model == &br_gravity_models[1]
                  ? atan2f(-target_world_m[0], target_world_m[1])
                  : atan2f(target_world_m[0], target_world_m[1]);
    }
    else
    {
        yaw = current_q[0];
    }

    a4 = a4_zero + current_q[1] - current_q[2] - current_q[3];
    wrist_x = -radius - l4 * cosf(a4);
    wrist_y = target_world_m[2] - l4 * sinf(a4);
    c2 = (wrist_x * wrist_x + wrist_y * wrist_y -
          l1 * l1 - welded_length * welded_length) /
         (2.0f * l1 * welded_length);
    if (c2 < -1.0f || c2 > 1.0f) return false;
    c2 = fmaxf(-1.0f, fminf(1.0f, c2));
    s2_abs = sqrtf(fmaxf(0.0f, 1.0f - c2 * c2));

    for (unsigned branch = 0U; branch < 2U; ++branch)
    {
        const float s2 = branch == 0U ? s2_abs : -s2_abs;
        const float welded_angle = atan2f(s2, c2);
        const float a1 = atan2f(wrist_y, wrist_x) -
                         atan2f(welded_length * s2,
                                l1 + welded_length * c2);
        const float q1 = a1 - a1_zero;
        const float welded_absolute = a1 + welded_angle;
        const float q2 = q1 - (welded_absolute - welded_zero);
        const float q3 = a4_zero + q1 - q2 - a4;

        candidate[branch][0] = br_wrap_deg(yaw * 180.0f / pi);
        candidate[branch][1] = br_wrap_deg(q1 * 180.0f / pi);
        candidate[branch][2] = br_wrap_deg(q2 * 180.0f / pi);
        candidate[branch][3] = br_wrap_deg(q3 * 180.0f / pi);
        candidate[branch][4] = current_q[4] * 180.0f / pi;
    }

    {
        const float current_q2_deg = current_q[2] * 180.0f / pi;
        const float error0 = fabsf(br_wrap_deg(candidate[0][2] - current_q2_deg));
        const float error1 = fabsf(br_wrap_deg(candidate[1][2] - current_q2_deg));
        best = prefer_small_second_rs00 ? (error1 < error0 ? 1U : 0U)
                                        : (error1 > error0 ? 1U : 0U);
    }
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        q_target_deg[i] = candidate[best][i];
    return true;
}
bool BR_GravityEvaluate(const br_gravity_model_t *model,
                        const float q[BR_ARM_DOF], bool grip,
                        float tau[BR_ARM_DOF], float *potential_j)
{
    br_vec2_t joint[4], com[4], tool;
    if (model == NULL || q == NULL || tau == NULL || model->link == NULL ||
        !isfinite(model->payload_mass_kg) || model->payload_mass_kg < 0.0f)
        return false;
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        if (!isfinite(q[i])) return false;
    for (unsigned i = 0U; i < BR_GRAVITY_LINK_COUNT; ++i)
        if (!isfinite(model->link_mass_kg[i]) || model->link_mass_kg[i] < 0.0f) return false;
    memset(tau, 0, sizeof(float) * BR_ARM_DOF);
    br_pose(q, joint, com, &tool);

    tau[1] = br_torque(joint[0], com[0], model->link_mass_kg[0], model->gravity_base) +
             br_torque(joint[0], com[1], model->link_mass_kg[1], model->gravity_base) +
             br_torque(joint[0], com[2], model->link_mass_kg[2], model->gravity_base) +
             br_torque(joint[0], com[3], model->link_mass_kg[3], model->gravity_base);
    tau[2] = br_torque(joint[1], com[1], model->link_mass_kg[1], model->gravity_base) +
             br_torque(joint[1], com[2], model->link_mass_kg[2], model->gravity_base) +
             br_torque(joint[1], com[3], model->link_mass_kg[3], model->gravity_base);
    tau[3] = br_torque(joint[3], com[3], model->link_mass_kg[3], model->gravity_base);
    if (grip && model->payload_mass_kg > 0.0f)
    {
        tau[1] += br_torque(joint[0], tool, model->payload_mass_kg, model->gravity_base);
        tau[2] += br_torque(joint[1], tool, model->payload_mass_kg, model->gravity_base);
        tau[3] += br_torque(joint[3], tool, model->payload_mass_kg, model->gravity_base);
    }
    tau[2] = -tau[2];
    tau[3] = -tau[3];
    tau[0] = 0.0f;
    tau[4] = 0.0f;
    if (potential_j != NULL)
    {
        *potential_j = 0.0f;
        for (unsigned i = 0U; i < 4U; ++i)
            *potential_j -= model->link_mass_kg[i] * model->gravity_base[1] * com[i].y;
        if (grip) *potential_j -= model->payload_mass_kg * model->gravity_base[1] * tool.y;
    }
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        if (!isfinite(tau[i])) return false;
    return true;
}
