#ifndef BOARD_CAN_H
#define BOARD_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define BOARD_CAN_CHASSIS_SPEED_ID   0x301U
#define BOARD_CAN_CHASSIS_CURRENT_ID 0x302U

typedef enum
{
    BOARD_CAN_COMMAND_NONE = 0U,
    BOARD_CAN_COMMAND_SPEED,
    BOARD_CAN_COMMAND_CURRENT
} board_can_command_type_t;

typedef struct
{
    int16_t values[4];
    board_can_command_type_t type;
} board_can_command_t;

HAL_StatusTypeDef Board_CAN_Send(uint32_t id, const uint8_t *data,
                                 uint8_t length);
bool Board_CAN_TakeCommand(board_can_command_t *command);
void Board_CAN_RxCallback(const CAN_RxHeaderTypeDef *header,
                          const uint8_t data[8]);

#endif
