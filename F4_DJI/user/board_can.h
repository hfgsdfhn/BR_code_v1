#ifndef __BOARD_CAN_H
#define __BOARD_CAN_H

#include "main.h"
#include "can.h"


extern volatile float PID_K[3][3];
extern volatile float leg_height_mm[4];
extern volatile float wheel_speed_mps[4];

void CAN_Parse_SmallBoard(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void Send_Feedback_Pos(CAN_HandleTypeDef *hcan, float pos_mm[4]);
void Send_Feedback_Speed(CAN_HandleTypeDef *hcan, float speed_mps[4]);
void SmallBoard_Send_PID(CAN_HandleTypeDef *hcan, uint8_t pid_id, float kp, float ki, float kd);
#endif
