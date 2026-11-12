#include "ak_motor.h"

#include <stddef.h>

#include "main.h"

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length);


static float AkMotor_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    {
        if (value > maximum)
        return maximum;
    }
    return value;
}

static float AkMotor_MoveToward(float current, float target, float step)
{
    if (current < target)
    {
        return AkMotor_Clamp(current + step, current, target);
    }
    if (current > target)
    {
        return AkMotor_Clamp(current - step, target, current);
    }
    return current;
}

static int16_t AkMotor_ReadI16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void AkMotor_WriteI32Be(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw >> 24U);
    data[1] = (uint8_t)(raw >> 16U);
    data[2] = (uint8_t)(raw >> 8U);
    data[3] = (uint8_t)raw;
}

HAL_StatusTypeDef AkMotor_SendServoSpeedCommand(const ak_motor_t *motor)
{
    uint8_t data[AK_MOTOR_SERVO_SPEED_DATA_LENGTH];
    float speed;
    int32_t speed_raw;
    uint32_t can_id;

    speed = AkMotor_Clamp(motor->command_speed, AK_MOTOR_SPEED_MIN,
                          AK_MOTOR_SPEED_MAX);
    speed_raw = (int32_t)speed;
    speed_raw = (int32_t)AkMotor_Clamp((float)speed_raw,
                                       AK_MOTOR_SPEED_MIN,
                                       AK_MOTOR_SPEED_MAX);
    can_id = motor->id | ((uint32_t)CAN_PACKET_SET_RPM << 8U);
    AkMotor_WriteI32Be(data, speed_raw);
    return Can_Transmit(motor->hcan, 1U, can_id, data,
                        AK_MOTOR_SERVO_SPEED_DATA_LENGTH);
}

HAL_StatusTypeDef AkMotor_SendCurrentCommand(const ak_motor_t *motor,
                                             int32_t current)
{
    uint8_t data[4];
    uint32_t can_id;

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    can_id = motor->id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8U);
    AkMotor_WriteI32Be(data, current);
    return Can_Transmit(motor->hcan, 1U, can_id, data, sizeof(data));
}

bool AkMotor_Init(ak_motor_t *motor, CAN_HandleTypeDef *hcan, uint8_t id)
{
    if ((motor == NULL) || (hcan == NULL) || (id < AK_MOTOR_MIN_ID)
        || (id > AK_MOTOR_MAX_ID))
    {
        return false;
    }

    motor->id = id;
    motor->hcan = hcan;
    motor->target_speed = 0.0f;
    motor->command_speed = 0.0f;
    motor->step.step_speed = 0.0f;
    motor->step.step_interval_ms = 0U;
    motor->step.last_step_tick = 0U;
    motor->step.step_started = false;
    motor->feedback.position = 0.0f;
    motor->feedback.speed = 0.0f;
    motor->feedback.torque = 0.0f;
    motor->feedback.temperature = 0U;
    motor->feedback.error_code = 0U;
    motor->feedback.valid = false;
    return true;
}

bool AkMotor_SetSpeed(ak_motor_t *motor, float speed)
{
    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return false;
    }

    motor->target_speed = AkMotor_Clamp(speed, AK_MOTOR_SPEED_MIN,
                                        AK_MOTOR_SPEED_MAX);
    motor->command_speed = motor->target_speed;
    motor->step.step_started = false;
    return AkMotor_SendServoSpeedCommand(motor) == HAL_OK;
}

bool AkMotor_SetCurrent(ak_motor_t *motor, int16_t current)
{
    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return false;
    }

    motor->step.step_started = false;
    return AkMotor_SendCurrentCommand(motor, (int32_t)current) == HAL_OK;
}

bool AkMotor_SetSpeedStep(ak_motor_t *motor, float target_rpm,
                          float step_rpm, uint32_t interval_ms)
{
    uint32_t now;

    if ((motor == NULL) || (motor->hcan == NULL) || (step_rpm <= 0.0f)
        || (interval_ms == 0U))
    {
        return false;
    }

    motor->target_speed = AkMotor_Clamp(target_rpm, AK_MOTOR_SPEED_MIN,
                                        AK_MOTOR_SPEED_MAX);
    motor->step.step_speed = step_rpm;
    motor->step.step_interval_ms = interval_ms;
    now = HAL_GetTick();

    if (!motor->step.step_started
        || ((uint32_t)(now - motor->step.last_step_tick) >= interval_ms))
    {
        motor->command_speed = AkMotor_MoveToward(motor->command_speed,
                                                   motor->target_speed,
                                                   step_rpm);
        motor->step.last_step_tick = now;
        motor->step.step_started = true;
        return AkMotor_SendServoSpeedCommand(motor) == HAL_OK;
    }

    return true;
}

bool AkMotor_ParseFeedback(ak_motor_t *motor,
                           const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8])
{
    uint32_t id;

    if ((motor == NULL) || (header == NULL) || (data == NULL)
        || (header->IDE != CAN_ID_EXT) || (header->DLC != 8U))
    {
        return false;
    }

    id = header->ExtId & 0xFFU;
    if (id != motor->id)
    {
        return false;
    }

    motor->feedback.position = (float)AkMotor_ReadI16Be(&data[0]) * 0.1f;
    motor->feedback.speed = (float)AkMotor_ReadI16Be(&data[2]) * 10.0f;
    motor->feedback.torque = (float)AkMotor_ReadI16Be(&data[4]) * 0.01f;
    motor->feedback.temperature = data[6];
    motor->feedback.error_code = data[7];
    motor->feedback.valid = true;
    return true;
}

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;

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

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &mailbox);
}
