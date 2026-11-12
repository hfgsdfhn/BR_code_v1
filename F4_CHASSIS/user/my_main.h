#ifndef MY_MAIN_H
#define MY_MAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "ak_motor.h"

extern ak_motor_t g_chassis_motors[4];

void My_Init(void);
void My_Loop(void);

#endif
