
#include "board_can.h"
#include "my_main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/**
 * @brief 发送底盘目标速度
 * @param hcan CAN句柄
 * @param vx X方向目标速度，m/s
 * @param vy Y方向目标速度，m/s
 * @param w 目标角速度，rad/s
 * @retval HAL状态
 */
HAL_StatusTypeDef Chassis_tx(CAN_HandleTypeDef *hcan,
                             float vx,
                             float vy,
                             float w)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;

    int16_t raw_vx;
    int16_t raw_vy;
    int16_t raw_w;

    if (hcan == NULL)
        return HAL_ERROR;

    /* 定点数转换，放大1000倍 */
    raw_vx = (int16_t)(vx * 1000.0f);
    raw_vy = (int16_t)(vy * 1000.0f);
    raw_w  = (int16_t)(w  * 1000.0f);

    /* 配置CAN帧头 */
    tx_header.StdId = BOARD_ID_SPEED;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;

    /* 打包三个速度，小端序 */
    tx_data[0] = (uint8_t)raw_vx;
    tx_data[1] = (uint8_t)(raw_vx >> 8);

    tx_data[2] = (uint8_t)raw_vy;
    tx_data[3] = (uint8_t)(raw_vy >> 8);

    tx_data[4] = (uint8_t)raw_w;
    tx_data[5] = (uint8_t)(raw_w >> 8);

    tx_data[6] = 0U;
    tx_data[7] = 0U;

    return HAL_CAN_AddTxMessage(hcan, &tx_header,
                                tx_data, &tx_mailbox);
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
