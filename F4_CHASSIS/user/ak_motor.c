#include "ak_motor.h"

#include <stddef.h>

#include "main.h"

static HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint8_t Header,
                                      uint32_t Id, uint8_t *data, uint8_t len);


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
    return CAN_Transmit(motor->hcan, 1, can_id, data,
                               AK_MOTOR_SERVO_SPEED_DATA_LENGTH);
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

bool AkMotor_ParseFeedback(ak_motor_t *motor, const BSP_CAN_Frame_t *frame)
{
    if ((motor == NULL) || (frame == NULL) || ((frame->id & 0xFFU) != motor->id)
        || (frame->dlc != AK_MOTOR_SERVO_FEEDBACK_LENGTH))
    {
        return false;
    }

    motor->feedback.position = (float)AkMotor_ReadI16Be(&frame->data[0]) * 0.1f;
    motor->feedback.speed = (float)AkMotor_ReadI16Be(&frame->data[2]) * 10.0f;
    motor->feedback.torque = (float)AkMotor_ReadI16Be(&frame->data[4]) * 0.01f;
    motor->feedback.temperature = frame->data[6];
    motor->feedback.error_code = frame->data[7];
    motor->feedback.valid = true;
    return true;
}

static HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint8_t Header,
                                      uint32_t Id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;
    uint32_t timeout = 0xFFFFFU;

    if ((hcan == NULL) || (data == NULL) || (Header > 1U))
    {
        return HAL_ERROR;
    }

    if (len > 8U)
    {
        len = 8U;
    }

    if (Header == 0U)
    {
        tx_header.StdId = Id;
        tx_header.IDE = CAN_ID_STD;
    }
    else
    {
        tx_header.ExtId = Id;
        tx_header.IDE = CAN_ID_EXT;
    }
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        if (--timeout == 0U)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &mailbox);
}
