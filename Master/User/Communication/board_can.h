#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#include "main.h"
#include "chassis.h"
#include "bsp_can.h"

#define BOARD_ID_SPEED 0x081U

HAL_StatusTypeDef Chassis_SendSpeed(FDCAN_HandleTypeDef *hfdcan,
                                    const chassis_speed_t *speed);
struct DJI_Board
{
    float PID_K[3][3];
    float leg_height_mm[4];
    float wheel_speed_mps[4];
};

extern DJI_Board DJI_Board_Target;
extern DJI_Board DJI_Board_FeedBack;
void init_board(void);
void CAN_Parse_SmallBoard(const can_frame_t *frame);
HAL_StatusTypeDef Send_Target_Speed(FDCAN_HandleTypeDef *hfdcan, float speed_mps[4]);
HAL_StatusTypeDef SmallBoard_Send_PID(FDCAN_HandleTypeDef *hfdcan, uint8_t pid_id, float kp, float ki, float kd);
HAL_StatusTypeDef Send_Target_Pos(FDCAN_HandleTypeDef *hfdcan, float pos_mm[4]);
#endif
