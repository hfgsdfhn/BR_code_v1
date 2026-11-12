
#include "board_can.h"
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
 * @brief 底盘小板CAN接收函数
 */
void Chassis_rx(CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    int16_t *p_target = NULL;

    if ((header == NULL) || (data == NULL))
        return;

    /* 检查数据帧格式 */
    if ((header->IDE != CAN_ID_STD) ||
        (header->RTR != CAN_RTR_DATA) ||
        (header->DLC != 8U))
        return;

    /* 根据帧ID选择目标数组和控制模式 */
    if (header->StdId == BOARD_ID_SPEED)
    {
        p_target = target_speed;
        target_mode_flag = BOARD_MODE_SPEED;
    }
    else if (header->StdId == BOARD_ID_CURRENT)
    {
        p_target = target_current;
        target_mode_flag = BOARD_MODE_CURRENT;
    }
    else
    {
        return;
    }

    /* 解析4个电机数据 */
    for (uint8_t i = 0U; i < BOARD_MOTOR_NUM; ++i)
    {
        uint16_t raw = (uint16_t)data[2U * i]
                     | ((uint16_t)data[2U * i + 1U] << 8);

        p_target[i] = (int16_t)raw;
    }
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

/**
 * @brief 主控CAN1发送底盘数据
 */
HAL_StatusTypeDef Board_CAN1_Send(
    uint16_t can_id,
    const int16_t data[BOARD_MOTOR_NUM])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    if (data == NULL)
        return HAL_ERROR;

    if (can_id != BOARD_ID_SPEED &&
        can_id != BOARD_ID_CURRENT)
        return HAL_ERROR;

    tx_header.StdId = can_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    for (uint8_t i = 0U; i < BOARD_MOTOR_NUM; i++)
    {
        uint16_t raw = (uint16_t)data[i];

        tx_data[2U * i] = (uint8_t)(raw & 0xFFU);
        tx_data[2U * i + 1U] = (uint8_t)(raw >> 8);
    }

    return HAL_CAN_AddTxMessage(
        &hcan1, &tx_header, tx_data, &tx_mailbox);
}


/**
 * @brief 主控发送DJI小板控制数据
 * @param lift_mode 四个3508升降控制位
 * @param action_mode 上台阶行动模式
 * @param target_speed 目标速度
 */
HAL_StatusTypeDef Board_DJI_Send(
    uint8_t lift_mode,
    uint8_t action_mode,
    float target_speed)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;
    uint32_t speed_raw;

    /* 限制升降控制为低四位 */
    lift_mode &= 0x0FU;

    tx_header.StdId = DJI_ID_CONTROL;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = lift_mode;
    tx_data[1] = action_mode;

    /* float32转为32位数据 */
    memcpy(&speed_raw, &target_speed, sizeof(float));

    /* 小端序排列 */
    tx_data[2] = (uint8_t)(speed_raw & 0xFFU);
    tx_data[3] = (uint8_t)((speed_raw >> 8) & 0xFFU);
    tx_data[4] = (uint8_t)((speed_raw >> 16) & 0xFFU);
    tx_data[5] = (uint8_t)((speed_raw >> 24) & 0xFFU);

    return HAL_CAN_AddTxMessage(
        &hcan1, &tx_header, tx_data, &tx_mailbox);
}



