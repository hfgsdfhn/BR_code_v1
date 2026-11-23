
#include "board_can.h"
#include "my_main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length);

HAL_StatusTypeDef Chassis_tx(CAN_HandleTypeDef *hcan,const float erpm[4])
{
    uint8_t tx_data[8] = {0};
    int16_t raw_speed[4];

    if (hcan == NULL || erpm == NULL)
        return HAL_ERROR;

    for (uint8_t i = 0; i < 4; i++)
    {
        /* ERPM轮子线速度 m/s */
        float speed = erpm[i] * 0.00012195f;

        /* 放大1000倍 */
        raw_speed[i] = (int16_t)(speed * 1000.0f);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        tx_data[i * 2] = (uint8_t)(raw_speed[i] & 0xFF);
        tx_data[i * 2 + 1] = (uint8_t)((raw_speed[i] >> 8) & 0xFF);
    }

    return Can_Transmit(hcan, 0U, 0x140U, tx_data, 8U);
}

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;
    uint32_t start_tick;

    if ((hcan == NULL) || (data == NULL) || (length > 8U) || (header > 1U))
    {
        return HAL_ERROR;
    }

    if (header == 0U)
    {
        if (id > 0x7FFU)
        {
            return HAL_ERROR;
        }
        tx_header.StdId = id;
        tx_header.IDE = CAN_ID_STD;
    }
    else
    {
        if (id > 0x1FFFFFFFU)
        {
            return HAL_ERROR;
        }
        tx_header.ExtId = id;
        tx_header.IDE = CAN_ID_EXT;
    }

    start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= 2U)
        {
            return HAL_TIMEOUT;
        }
    }

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &mailbox);
}

/**
 * @brief 底盘CAN接收回调
 * @param header CAN接收帧头
 * @param data CAN接收数据
 */
void Chassis_rx(CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    int16_t raw_vx;
    int16_t raw_vy;
    int16_t raw_w;

    if ((header == NULL) || (data == NULL))	
        return;

    /* 检查数据帧格式 */
    if ((header->IDE != CAN_ID_STD) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U))
        return;

    if (header->StdId != BOARD_ID_SPEED)
        return;

    /* 解析三个速度 */
    raw_vx = (int16_t)((uint16_t)data[0] |
                      ((uint16_t)data[1] << 8));

    raw_vy = (int16_t)((uint16_t)data[2] |
                      ((uint16_t)data[3] << 8));

    raw_w = (int16_t)((uint16_t)data[4] |
                     ((uint16_t)data[5] << 8));

    /* 转换为国际单位制 */
    chassis_target_speed.vx = (float)raw_vx / 1000.0f;
    chassis_target_speed.vy = (float)raw_vy / 1000.0f;
    chassis_target_speed.vw = (float)raw_w / 1000.0f;
}
