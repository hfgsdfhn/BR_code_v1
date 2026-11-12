#include "board_can.h"

#include <string.h>

volatile uint8_t dji_lift_mode = 0U;
volatile uint8_t dji_action_mode = 0U;
volatile float dji_target_speed = 0.0f;

void Board_DJI_Rx(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8])
{
    uint32_t speed_raw;
    float target_speed;

    if ((rx_header == NULL) || (rx_data == NULL))
    {
        return;
    }

    /* 检查数据帧格式 */
    if ((rx_header->IDE != CAN_ID_STD)
        || (rx_header->RTR != CAN_RTR_DATA)
        || (rx_header->StdId != DJI_ID_CONTROL)
        || (rx_header->DLC != 8U))
    {
        return;
    }

    /* 解析控制模式 */
    dji_lift_mode = rx_data[0] & 0x0FU;
    dji_action_mode = rx_data[1];

    /* 解析浮点数速度 */
    speed_raw = (uint32_t)rx_data[2]
              | ((uint32_t)rx_data[3] << 8U)
              | ((uint32_t)rx_data[4] << 16U)
              | ((uint32_t)rx_data[5] << 24U);

    memcpy(&target_speed, &speed_raw, sizeof(target_speed));

    dji_target_speed = target_speed;
}
