#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "dji_motor.h"
#include "main.h"
#include "pid.h"

// 减速比 19.2，周长 60mm
#define encoder_to_mm 0.00038147f 
#define RPM_TO_MPS_DIV 0.00016357f
#define RPM_TO_SPEED 0.0000582f
// 统一变量名
extern pid_t leg_position_pid[4];
extern pid_t leg_speed_pid[4];
extern pid_t wheel_speed_pid[4];

extern dji_motor_t M2006[4];
extern dji_motor_t M3508[4];

void DJI_OnData(const CAN_RxHeaderTypeDef *header, const uint8_t *data);
void My_Init(void);
void My_Loop(void);

#endif