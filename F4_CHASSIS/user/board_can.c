#include "board_can.h"

#include <stddef.h>

#include "can.h"

static volatile board_can_command_t s_board_can_command = {0};
static volatile uint8_t s_board_can_command_pending = 0U;

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;

    if ((hcan == NULL) || (data == NULL) || (length > 8U) || (header != 0U)
        || (id > 0x7FFU))
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &mailbox);
}

HAL_StatusTypeDef Board_CAN_Send(uint32_t id, const uint8_t *data,
                                 uint8_t length)
{
    return Can_Transmit(&hcan1, 0U, id, data, length);
}

bool Board_CAN_TakeCommand(board_can_command_t *command)
{
    uint32_t interrupt_state;

    if (command == NULL)
    {
        return false;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    if (s_board_can_command_pending == 0U)
    {
        __set_PRIMASK(interrupt_state);
        return false;
    }

    command->type = s_board_can_command.type;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        command->values[i] = s_board_can_command.values[i];
    }
    s_board_can_command_pending = 0U;
    __set_PRIMASK(interrupt_state);
    return true;
}

void Board_CAN_RxCallback(const CAN_RxHeaderTypeDef *header,
                          const uint8_t data[8])
{
    if ((header == NULL) || (data == NULL) || (header->IDE != CAN_ID_STD)
        || (header->DLC != 8U))
    {
        return;
    }

    if (header->StdId == BOARD_CAN_CHASSIS_SPEED_ID)
    {
        s_board_can_command.type = BOARD_CAN_COMMAND_SPEED;
    }
    else if (header->StdId == BOARD_CAN_CHASSIS_CURRENT_ID)
    {
        s_board_can_command.type = BOARD_CAN_COMMAND_CURRENT;
    }
    else
    {
        return;
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        uint16_t value = (uint16_t)data[2U * i]
                         | ((uint16_t)data[2U * i + 1U] << 8U);
        s_board_can_command.values[i] = (int16_t)value;
    }
    s_board_can_command_pending = 1U;
}
