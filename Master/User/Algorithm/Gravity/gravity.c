#include "gravity.h"

#include <math.h>
#include <string.h>

#define BR_PI 3.14159265358979323846f
#define BR_DEG_TO_RAD (BR_PI / 180.0f)

/* 2-D drawing model, units: metre.
 * x is right, y is up; positive angle is counter-clockwise from +x.
 * When all motor commands are 0 deg, the four absolute ground angles
 * are 25, 155, 195 and 190 deg respectively.
 * The welded 6 cm + 21.5 cm bar is driven by the second RS00, so both
 * segments receive q[2]. The final 8 cm segment receives q[3].
 */
static const float br_link_37_m = 0.370000f;
static const float br_link_6_m = 0.060000f;
static const float br_link_21_5_m = 0.215000f;
static const float br_link_8_m  = 0.080000f;
/* Absolute member angles measured from the horizontal ground at motor command 0 deg. */
static const float br_command_zero_ground_deg[4] = {25.0f, 155.0f, 195.0f, 190.0f};

/* Masses and COM distances taken from br.zip. The distances are projected
 * onto the corresponding 2-D member shown in the drawing. */
static const float br_mass_003_kg = 0.670000000000f;
static const float br_mass_004_kg = 0.072000000000f;
static const float br_mass_005_kg = 0.430000000000f;
static const float br_mass_006_kg = 1.008000000000f;
static const float br_com_37_m = 0.21244607f;
static const float br_com_6_m = 0.049223881f;
static const float br_com_21_5_m = 0.049902387f;
static const float br_com_8_m = 0.080391522f;

static const float br_tool_offset_m[3] = {0.0f, 0.0f, 0.0f};
static const float br_identity[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

static br_gravity_link_t br_g_links[BR_GRAVITY_LINK_COUNT] =
{
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = br_mass_003_kg, .com_m = {br_com_37_m, 0.0f, 0.0f}, .joint_index = 1U},
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = br_mass_004_kg, .com_m = {br_com_6_m, 0.0f, 0.0f}, .joint_index = 2U},
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = br_mass_005_kg, .com_m = {br_com_21_5_m, 0.0f, 0.0f}, .joint_index = 2U},
    {.origin = {0.0f, 0.0f, 0.0f}, .fixed_rotation = br_identity, .axis = {0.0f, 0.0f, 1.0f},
     .mass_kg = br_mass_006_kg, .com_m = {br_com_8_m, 0.0f, 0.0f}, .joint_index = 3U}
};

br_gravity_model_t br_gravity_models[2] =
{
    {.link = br_g_links, .gravity_base = {0.0f, -9.80665f, 0.0f},
     .payload_mass_kg = 0.174f, .payload_com_m = {0.0f, 0.0f, 0.0f},
     .tool_offset_m = br_tool_offset_m, .calibrated = true},
    {.link = br_g_links, .gravity_base = {0.0f, -9.80665f, 0.0f},
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
    br_vec2_t joint[4], com[4], tool;
    if (model == NULL || q == NULL || end_world_m == NULL || model->link == NULL)
        return false;
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        if (!isfinite(q[i])) return false;
    br_pose(q, joint, com, &tool);
    end_world_m[0] = tool.x;
    end_world_m[1] = tool.y;
    end_world_m[2] = 0.0f;
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

    memset(tau, 0, sizeof(float) * BR_ARM_DOF);
    br_pose(q, joint, com, &tool);

    /* q[1]: lower RS00, q[2]: upper RS00/welded 6+21.5 cm bar, q[3]: J4310. */
    tau[1] = br_torque(joint[0], com[0], br_mass_003_kg, model->gravity_base) +
             br_torque(joint[0], com[1], br_mass_004_kg, model->gravity_base) +
             br_torque(joint[0], com[2], br_mass_005_kg, model->gravity_base) +
             br_torque(joint[0], com[3], br_mass_006_kg, model->gravity_base);
    tau[2] = br_torque(joint[1], com[1], br_mass_004_kg, model->gravity_base) +
             br_torque(joint[1], com[2], br_mass_005_kg, model->gravity_base) +
             br_torque(joint[1], com[3], br_mass_006_kg, model->gravity_base);
    tau[3] = br_torque(joint[3], com[3], br_mass_006_kg, model->gravity_base);

    if (grip && model->payload_mass_kg > 0.0f)
    {
        tau[1] += br_torque(joint[0], tool, model->payload_mass_kg, model->gravity_base);
        tau[2] += br_torque(joint[1], tool, model->payload_mass_kg, model->gravity_base);
        tau[3] += br_torque(joint[3], tool, model->payload_mass_kg, model->gravity_base);
    }

    /* Motor-axis convention: RS00_2 and J4310 use the opposite sign. */
    tau[2] = -tau[2];
    tau[3] = -tau[3];
    tau[0] = 0.0f;
    tau[4] = 0.0f;
    if (potential_j != NULL)
    {
        *potential_j = 0.0f;
        for (unsigned i = 0U; i < 4U; ++i)
            *potential_j -= model->link[i].mass_kg * model->gravity_base[1] * com[i].y;
        if (grip) *potential_j -= model->payload_mass_kg * model->gravity_base[1] * tool.y;
    }
    for (unsigned i = 0U; i < BR_ARM_DOF; ++i)
        if (!isfinite(tau[i])) return false;
    return true;
}
