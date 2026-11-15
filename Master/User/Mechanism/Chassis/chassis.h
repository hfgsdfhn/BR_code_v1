#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote.h"

#define CHASSIS_JOYSTICK_MAX_LINEAR_MPS       2.0f
#define CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S    2.0f
#define CHASSIS_JOYSTICK_CENTER               8
#define CHASSIS_JOYSTICK_DEADBAND             1
#define CHASSIS_LINEAR_SPEED_STEP_MPS         0.01f
#define CHASSIS_ANGULAR_SPEED_STEP_RAD_S      0.02f

#define M3508_LEG_MOTOR_ID_MIN 5U
#define M3508_LEG_MOTOR_ID_MAX 8U

typedef enum
{
    M3508_POS_PID = 0,
    M3508_SPEED_PID,
    M2006_SPEED_PID
}DJI_PID_type_e;

typedef struct DJI_Board DJI_Board;

typedef struct
{
    float vx;
    float vy;
    float vw;
}chassis_speed_t;

void leg_M3508_ctrl(DJI_Board *dji_board,
                    DJI_PID_type_e pid_type,
                    uint8_t motor_id,
                    float ki,
                    float kd,
                    float target);
void joy_to_mps(const remote_t *remote, chassis_speed_t *speed);

#endif
