
#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#include "main.h"
#include <stdint.h>

/* 底盘小板通信ID */
#define BOARD_ID_SPEED       0x081U
#define BOARD_ID_CURRENT     0x082U

/* DJI小板通信ID */
#define DJI_ID_CONTROL       0x100U

/* 电机数量 */
#define BOARD_MOTOR_NUM      4U

/* DJI升降控制 */
#define DJI_LIFT_DOWN        0U
#define DJI_LIFT_UP          1U

/* DJI行动模式 */
#define DJI_MODE_NORMAL      0U
#define DJI_MODE_CLIMB       1U

/* 底盘控制模式 */
typedef enum
{
    BOARD_MODE_SPEED = 0,
    BOARD_MODE_CURRENT
} board_mode_e;

extern volatile board_mode_e target_mode_flag;

/* 底盘接收数据 */
extern int16_t target_speed[BOARD_MOTOR_NUM];
extern int16_t target_current[BOARD_MOTOR_NUM];

/* DJI接收数据 */
extern uint8_t dji_lift_mode;
extern uint8_t dji_action_mode;
extern float dji_target_speed;

/* 底盘CAN接收 */
void Chassis_rx(CAN_RxHeaderTypeDef *header, uint8_t data[8]);

/* 底盘CAN发送 */
HAL_StatusTypeDef Board_CAN1_Send(
    uint16_t can_id,
    const int16_t data[BOARD_MOTOR_NUM]);

/* DJI CAN发送 */
HAL_StatusTypeDef Board_DJI_Send(
    uint8_t lift_mode,
    uint8_t action_mode,
    float target_speed);

/* DJI CAN接收 */
void Board_DJI_Rx(CAN_HandleTypeDef *hcan);

#endif
