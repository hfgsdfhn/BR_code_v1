#include "board_can.h"

#include <string.h>

volatile uint8_t dji_lift_mode = 0U;
volatile uint8_t dji_action_mode = 0U;
volatile float dji_target_speed = 0.0f;

void Board_DJI_Rx(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan == NULL)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        uint32_t speed_raw;
        float target_speed;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                 &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD)
            || (rx_header.RTR != CAN_RTR_DATA)
            || (rx_header.StdId != DJI_ID_CONTROL)
            || (rx_header.DLC != 8U))
        {
            continue;
        }

        dji_lift_mode = rx_data[0] & 0x0FU;
        dji_action_mode = rx_data[1];

        speed_raw = (uint32_t)rx_data[2]
                  | ((uint32_t)rx_data[3] << 8U)
                  | ((uint32_t)rx_data[4] << 16U)
                  | ((uint32_t)rx_data[5] << 24U);
        memcpy(&target_speed, &speed_raw, sizeof(target_speed));
        dji_target_speed = target_speed;
    }
}
