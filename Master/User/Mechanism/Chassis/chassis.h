#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote.h"

#define CHASSIS_JOYSTICK_MAX_LINEAR_MPS       2.0f
#define CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S    2.0f
#define CHASSIS_JOYSTICK_CENTER               8
#define CHASSIS_JOYSTICK_DEADBAND             1
#define CHASSIS_LINEAR_SPEED_STEP_MPS         0.01f
#define CHASSIS_ANGULAR_SPEED_STEP_RAD_S      0.02f

typedef struct
{
    float vx;
    float vy;
    float vw;
}chassis_speed_t;


void joy_to_mps(const remote_t *remote, chassis_speed_t *speed);

#endif
