#include "dji_motor.h"

#include <stddef.h>

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length);


//读取16位大端有符号整数
static int16_t DjiMotor_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

//写入16位大端有符号整数
static void DjiMotor_WriteI16Be(uint8_t *data, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

void DjiMotor_Init(dji_motor_t *motor, CAN_HandleTypeDef *can, uint8_t id, dji_motor_type_t type, int8_t dir)
{
    // 方向必须是 +1 或 -1，避免非单位值缩放目标电流。
    if ((motor == NULL) || (can == NULL) || (id < DJI_MOTOR_MIN_ID)
        || (id > DJI_MOTOR_MAX_ID) || (type > DJI_MOTOR_TYPE_M3508)
        || ((dir != 1) && (dir != -1)))
    {
        return;
    }

    motor->id = id;
    motor->type = type;
    motor->dir = dir;
    motor->encoder = 0U;
    motor->last_encoder = 0U;
    motor->encoder_turns = 0;
    motor->total_encoder = 0;
    motor->offset = 0;
    motor->encoder_ready = 0U;
    motor->speed_rpm = 0;
    motor->current = 0;
    motor->temperature = 0U;
    motor->target_current = 0;
    motor->can = can;
}

void DjiMotor_SetCurrent(dji_motor_t *motor, int16_t current)
{
    int16_t limit;

    if (motor == NULL)
    {
        return;
    }

    limit = (motor->type == DJI_MOTOR_TYPE_M2006) ? DJI_M2006_MAX_CURRENT
                                                   : DJI_M3508_MAX_CURRENT;

    // 在限制范围之前，先将上层控制的电流乘上方向
    int32_t actual_current = (int32_t)current * motor->dir;

    if (actual_current > limit)
    {
        actual_current = limit;
    }
    else if (actual_current < -limit)
    {
        actual_current = -limit;
    }

    motor->target_current = (int16_t)actual_current;
}

void DjiMotor_ParseFeedback(dji_motor_t *motor, const uint8_t *data)
{
    uint16_t encoder;
    int32_t raw_total;
    int32_t half_range;
    int32_t full_range;

    if ((motor == NULL) || (data == NULL)) return;

    encoder = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);

    if (motor->type == DJI_MOTOR_TYPE_M3508) {
        full_range = 8192; half_range = 4096;
    } else {
        full_range = 4096; half_range = 2048;
    }

    if (motor->encoder_ready != 0U)
    {
        int32_t delta = (int32_t)encoder - (int32_t)motor->last_encoder;

        if (delta > half_range) {
            motor->encoder_turns--;
        } 
        else if (delta < -half_range) {
            motor->encoder_turns++;
        }

        // 绝对位置解算
        raw_total = (motor->encoder_turns * full_range) + (int32_t)encoder;

        // 纯整数方向修正 + Offset 补偿
        if (motor->dir > 0) {
            motor->total_encoder = raw_total - motor->offset;
        } else {
            motor->total_encoder = -raw_total - motor->offset;
        }
    }
    else
    {
        motor->encoder_ready = 1U;
        motor->encoder_turns = 0;
        // 第一次收到数据
        raw_total = (int32_t)encoder;
        // 如果是第一次，offset 可能还没设置，如果没设默认为 0
        if (motor->dir > 0) {
            motor->total_encoder = raw_total - motor->offset;
        } else {
            motor->total_encoder = -raw_total - motor->offset; 
        }
    }
    
    motor->encoder = encoder;
    motor->last_encoder = encoder;
    
    // 速度和电流解析 (纯整数，性能更好)
    int16_t raw_speed = DjiMotor_ReadI16Be(&data[2]);
    motor->speed_rpm = (motor->dir > 0) ? (float)raw_speed : -(float)raw_speed;
    
    int16_t raw_current = DjiMotor_ReadI16Be(&data[4]);
    motor->current = (motor->dir > 0) ? (float)raw_current : -(float)raw_current;
    
    motor->temperature = data[6];
}

void DjiMotor_SetZero(dji_motor_t *motor, uint8_t number)
{
    for (uint8_t i = 0; i < number; i++)
    {
        if (motor[i].encoder_ready != 0U) 
        {
            int32_t full_range = (motor[i].type == DJI_MOTOR_TYPE_M3508) ? 8192 : 4096;
            int32_t raw_total = (motor[i].encoder_turns * full_range) + (int32_t)motor[i].encoder;
            
            if (motor[i].dir > 0) {
                motor[i].offset = raw_total;
            } else {
                motor[i].offset = -raw_total;
            }
            
            // 强制刷新一下 total_encoder
            motor[i].total_encoder = 0; 
        }
    }
}

/**
 * @brief 发送电机目标电流
 * 
 * @param motor 指向一个连续ID组的电机数组: 1-4 或 5-8
 * @param number 电机数量
 */
bool DjiMotor_SendGroup(dji_motor_t *motor, uint8_t number)
{
    uint8_t data[DJI_MOTOR_DATA_LENGTH] = {0};
    uint8_t index;
    uint32_t can_id;

    if ((motor == NULL) || (motor->can == NULL) || (number == 0U)
        || (number > 4U) || (motor->id < DJI_MOTOR_MIN_ID)
        || (motor->id > DJI_MOTOR_MAX_ID))
    {
        return false;
    }

    can_id = (motor->id <= 4U) ? DJI_MOTOR_CONTROL_ID_1_4
                               : DJI_MOTOR_CONTROL_ID_5_8;
    for (index = 0U; index < number; index++)
    {
        DjiMotor_WriteI16Be(&data[index * 2U], motor[index].target_current);
    }

    /* header=0 表示标准帧 */
    return Can_Transmit(motor->can, 0U, can_id, data,
                        DJI_MOTOR_DATA_LENGTH) == HAL_OK;
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

    /* 最多等待2ms，直到发送邮箱空闲 */
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
