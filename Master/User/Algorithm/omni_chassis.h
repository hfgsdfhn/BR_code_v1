
#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#define WHEEL_RADIUS     0.155f
#define CHASSIS_WIDTH    0.558f
#define CHASSIS_LENGTH   0.558f
#define sin45            0.7071f
#define CHASSIS_RADIUS   ((CHASSIS_WIDTH / 2.0f + CHASSIS_LENGTH / 2.0f) * sin45)

typedef struct {
    float vx;
    float vy;
    float vw;
} chassis_speed_t;

typedef struct {
    float lf;
    float rf;
    float lr;
    float rr;
} wheel_speed_t;

typedef struct {
    float x;
    float y;
    float yaw;
} chassis_position_t;

typedef struct {
    wheel_speed_t   wheel;
    chassis_speed_t speed;
    chassis_position_t position;
} chassis_state_t;

typedef struct {
    chassis_state_t state;
    chassis_speed_t target_speed;
    wheel_speed_t   target_wheel;
    chassis_speed_t command_speed;
    chassis_speed_t last_speed;
    chassis_position_t target_position;
} omni_chassis_t;

extern omni_chassis_t chassis;

void chassis_set_speed(void);
void chassis_get_speed(void);
void Chassis_BodyToWorld(void);
float Chassis_WheelSpeedToYawRate(float wheel_speed);

#endif
