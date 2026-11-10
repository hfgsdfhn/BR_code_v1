#ifndef BR_GRAVITY_H
#define BR_GRAVITY_H
#include <stdbool.h>
#define BR_ARM_DOF 5U
/* Joint order: root, shoulder RS00, elbow RS00, wrist J4310, tool rotation RS05.
 * origin / fixed_rotation map a joint's zero frame into its parent link frame.
 * axis is a unit vector in that zero frame; COM is in the rotating link frame.
 * Include every fixed child part (motor housing, brackets, tool) in link mass/COM.
 */
typedef struct
{
    float origin[3], fixed_rotation[3][3], axis[3];
    float mass_kg, com_m[3];
} br_gravity_link_t;
typedef struct
{
    br_gravity_link_t link[BR_ARM_DOF];
    float gravity_base[3];                   /* gravity acceleration expressed in the arm base */
    float payload_mass_kg, payload_com_m[3]; /* in the last rotating link */
    bool calibrated;
} br_gravity_model_t;
/* Returns holding torque +dU/dq; gravity itself applies -dU/dq.
 * The payload is included only when grip=true. Never uses the desired pose.
 * Uncalibrated/invalid models return false with all outputs zero. */
bool BR_GravityEvaluate(const br_gravity_model_t *model, const float q[BR_ARM_DOF], bool grip, float tau[BR_ARM_DOF],
                        float *potential_j);
#endif
