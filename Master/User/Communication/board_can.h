/*
 * @file board_can.h
 * @brief board_can模块的用户接口与实现。
 */

#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#include "bsp_can.h"
#include <stdint.h>

/* 底盘小板通信ID */
#define BOARD_ID_SPEED       0x081U
#define BOARD_ID_CURRENT     0x082U

#define DJI_ID_CONTROL       0x100U

/* 电机数量 */
#define BOARD_MOTOR_NUM      4U

#define DJI_LIFT_DOWN        0U
#define DJI_LIFT_UP          1U

#define DJI_MODE_NORMAL      0U
#define DJI_MODE_CLIMB       1U

/* 底盘控制模式 */
typedef enum
{
    BOARD_MODE_SPEED = 0, /* 当前工作模式或状态。 */
    BOARD_MODE_CURRENT /* 当前工作模式或状态。 */
} board_mode_e;

extern volatile board_mode_e target_mode_flag;

/* 底盘接收数据 */
extern int16_t target_speed[BOARD_MOTOR_NUM];
extern int16_t target_current[BOARD_MOTOR_NUM];

extern uint8_t dji_lift_mode; /* 当前工作模式或状态。 */
extern uint8_t dji_action_mode; /* 当前工作模式或状态。 */
extern float dji_target_speed;

/* 功能：处理Chassis_rx对应的输入并更新相关状态。 */
void Chassis_rx(CAN_RxHeaderTypeDef *header, uint8_t data[8]);

/* 功能：发送Board_CAN1_Send对应的数据或控制指令。 */
HAL_StatusTypeDef Board_CAN1_Send(
    uint16_t can_id,
    const int16_t data[BOARD_MOTOR_NUM]);

/* 功能：发送Board_DJI_Send对应的数据或控制指令。 */
HAL_StatusTypeDef Board_DJI_Send(
    uint8_t lift_mode, /* 当前工作模式或状态。 */
    uint8_t action_mode, /* 当前工作模式或状态。 */
    float target_speed);

/* 功能：处理Board_DJI_Rx对应的输入并更新相关状态。 */
void Board_DJI_Rx(CAN_HandleTypeDef *hcan);

#endif
