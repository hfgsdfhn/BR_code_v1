#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#include "main.h"
#include "chassis.h"

#define BOARD_ID_SPEED 0x081U

HAL_StatusTypeDef Chassis_SendSpeed(FDCAN_HandleTypeDef *hfdcan,
                                    const chassis_speed_t *speed);

/* 功能：处理Board_DJI_Rx对应的输入并更新相关状态。 */
void Board_DJI_Rx(CAN_HandleTypeDef *hcan);

#endif
