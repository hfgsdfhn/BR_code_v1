
#include "board_can.h"
#include "my_main.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/* 底盘目标数据 */
int16_t target_speed[BOARD_MOTOR_NUM] = {0};
int16_t target_current[BOARD_MOTOR_NUM] = {0};
volatile board_mode_e target_mode_flag = BOARD_MODE_SPEED;

/* DJI接收数据 */
uint8_t dji_lift_mode = 0U;
uint8_t dji_action_mode = 0U;
float dji_target_speed = 0.0f;

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

/**
 * @brief DJI小板CAN接收函数
 */
void Board_DJI_Rx(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t speed_raw;

    if (hcan == NULL)
        return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                 &rx_header, rx_data) != HAL_OK)
            break;

        if (rx_header.IDE != CAN_ID_STD ||
            rx_header.RTR != CAN_RTR_DATA ||
            rx_header.StdId != DJI_ID_CONTROL ||
            rx_header.DLC != 8U)
            continue;

        dji_lift_mode = rx_data[0] & 0x0FU;
        dji_action_mode = rx_data[1];

        /* 小端序还原float32 */
        speed_raw = (uint32_t)rx_data[2]
                  | ((uint32_t)rx_data[3] << 8)
                  | ((uint32_t)rx_data[4] << 16)
                  | ((uint32_t)rx_data[5] << 24);

        memcpy(&dji_target_speed, &speed_raw, sizeof(float));
    }
}

