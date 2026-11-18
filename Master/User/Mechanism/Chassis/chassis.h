#ifndef CHASSIS_H
#define CHASSIS_H
#include "remote.h"
#include "pid.h"

#define CHASSIS_JOYSTICK_MAX_LINEAR_MPS       2.0f
#define CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S    2.0f
#define CHASSIS_CONTROL_PERIOD_S              0.002f
#define CHASSIS_RAD_TO_DEG                    57.2957795f
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

typedef enum
{
    all = 0,
    qian,
    hou,
}DJI_leg_pos_e;

typedef struct DJI_Board DJI_Board;

typedef struct
{
    float vx;
    float vy;
    float vw;
}chassis_speed_t;

typedef struct send_to_chassis_t send_to_chassis_t;

void chassis_init();
void M2006_wheel_ctrl(float speed);
void joy_to_mps(send_to_chassis_t *send);
void key_to_chassis(send_to_chassis_t *send);
void set_target_position(float posa,float front, float behind);

extern pid_t yaw_pid;

#endif
