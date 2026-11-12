/*
 * @file board_can.c
 * @brief board_can模块的用户接口与实现。
 */

#include "board_can.h"

#include <stddef.h>
#include <string.h>

/* 底盘目标数据 */
int16_t target_speed[BOARD_MOTOR_NUM] = {0};
int16_t target_current[BOARD_MOTOR_NUM] = {0};
volatile board_mode_e target_mode_flag = BOARD_MODE_SPEED;

static HAL_StatusTypeDef Board_CAN1_SendFrame(uint32_t can_id,
                                               const uint8_t data[8])
{
    can_frame_t frame = {0};

    frame.hfdcan = &hfdcan1;
    frame.id = can_id;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.length = 8U;
    memcpy(frame.data, data, sizeof(frame.data));

    return BspCan_Send(&frame) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 主控CAN1发送底盘数据
 */
HAL_StatusTypeDef Board_CAN1_Send(
    uint16_t can_id,
    const int16_t data[BOARD_MOTOR_NUM])
{
    uint8_t tx_data[8] = {0};

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (can_id != BOARD_ID_SPEED &&
        can_id != BOARD_ID_CURRENT)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < BOARD_MOTOR_NUM; i++)
    {
        uint16_t raw = (uint16_t)data[i];

        tx_data[2U * i] = (uint8_t)(raw & 0xFFU);
        tx_data[2U * i + 1U] = (uint8_t)(raw >> 8);
    }

/* 功能：执行 HAL_CAN_AddTxMessage 模块操作并维护相关状态。 */
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
    uint8_t tx_data[8] = {0};
    uint32_t speed_raw;

    /* 限制升降控制为低四位 */
    lift_mode &= 0x0FU;

    tx_data[0] = lift_mode;
    tx_data[1] = action_mode;

    /* 模块实现细节。 */
    memcpy(&speed_raw, &target_speed, sizeof(float));

    /* 小端序排列 */
    tx_data[2] = (uint8_t)(speed_raw & 0xFFU);
    tx_data[3] = (uint8_t)((speed_raw >> 8) & 0xFFU);
    tx_data[4] = (uint8_t)((speed_raw >> 16) & 0xFFU);
    tx_data[5] = (uint8_t)((speed_raw >> 24) & 0xFFU);

/* 功能：执行 HAL_CAN_AddTxMessage 模块操作并维护相关状态。 */
    return HAL_CAN_AddTxMessage(
        &hcan1, &tx_header, tx_data, &tx_mailbox);
}
