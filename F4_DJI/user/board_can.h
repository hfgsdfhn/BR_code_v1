#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#include "main.h"

#define DJI_ID_CONTROL 0x100U

extern volatile uint8_t dji_lift_mode;
extern volatile uint8_t dji_action_mode;
extern volatile float dji_target_speed;

void Board_DJI_Rx(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8]);

#endif
