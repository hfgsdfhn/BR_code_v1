/*
 * @file board_can.c
 * @brief board_can模块的用户接口与实现。
 */

#include "board_can.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/* 底盘目标数据 */
int16_t target_speed[BOARD_MOTOR_NUM] = {0};
int16_t target_current[BOARD_MOTOR_NUM] = {0};
volatile board_mode_e target_mode_flag = BOARD_MODE_SPEED;

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

/* 功能：执行 HAL_CAN_AddTxMessage 函数对应的模块操作。 */
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

    /* 代码说明：相关模块实现。 */
    memcpy(&speed_raw, &target_speed, sizeof(float));

    /* 小端序排列 */
    tx_data[2] = (uint8_t)(speed_raw & 0xFFU);
    tx_data[3] = (uint8_t)((speed_raw >> 8) & 0xFFU);
    tx_data[4] = (uint8_t)((speed_raw >> 16) & 0xFFU);
    tx_data[5] = (uint8_t)((speed_raw >> 24) & 0xFFU);

/* 功能：执行 HAL_CAN_AddTxMessage 函数对应的模块操作。 */
    return HAL_CAN_AddTxMessage(
        &hcan1, &tx_header, tx_data, &tx_mailbox);
}
