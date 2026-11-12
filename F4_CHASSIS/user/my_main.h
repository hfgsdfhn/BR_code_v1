#ifndef MY_MAIN_H
#define MY_MAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "ak_motor.h"
#include "chassis_kinematics.h"

#define torque_to_current 12.82f
extern ak_motor_t g_chassis_motors[4];
extern chassis_speed_t chassis_actual_speed;
extern ak_motor_t g_chassis_motors[4];

void My_Init(void);
void My_Loop(void);

#endif
