#ifndef MY_MAIN_H
#define MY_MAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "ak_motor.h"
#include "chassis_kinematics.h"

#define torque_to_current 12.82f

extern chassis_speed_t chassis_target_speed;
extern chassis_speed_t chassis_actual_speed;
extern chassis_force_t chassis_target_force;

extern volatile uint8_t target_mode;

extern ak_motor_t g_chassis_motors[4];
extern float feedback_motor_speed[4];   /* 电机反馈转速  */
extern float target_motor_speed[4];     /* 速度模式输出  */
extern float target_motor_torque[4];    /* 电流模式输出  */

void My_Init(void);
void My_Loop(void);

#endif /* MY_MAIN_H */