#ifndef AK_MOTOR_H
#define AK_MOTOR_H /* AK 电机驱动头文件保护宏 */

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define AK_MOTOR_MIN_ID                  1U     /* 有效电机 ID 最小值 */
#define AK_MOTOR_MAX_ID                  0x7FU  /* 有效电机 ID 最大值 */

//速度限幅
#define AK_MOTOR_SPEED_MIN               (-100000.0f) /* 允许的最小电机转速，单位 eRPM */
#define AK_MOTOR_SPEED_MAX               100000.0f    /* 允许的最大电机转速，单位 eRPM */
#define AK_MOTOR_SERVO_SPEED_DATA_LENGTH 4U
#define AK_MOTOR_SERVO_FEEDBACK_LENGTH 8U

typedef enum
{
    CAN_PACKET_SET_DUTY = 0, /* 占空比模式。 */
    CAN_PACKET_SET_CURRENT, /* 电流环模式。 */
    CAN_PACKET_SET_CURRENT_BRAKE, /* 电流刹车模式。 */
    CAN_PACKET_SET_RPM, /* 转速模式。 */
    CAN_PACKET_SET_POS, /* 位置模式。 */
    CAN_PACKET_SET_ORIGIN_HERE, /* 设置原点模式。 */
    CAN_PACKET_SET_POS_SPD, /* 位置速度环模式。 */
    CAN_PACKET_SET_mit = 8 /* 运控控制模式。 */
} CAN_PACKET_ID;

typedef struct
{
    float position;
    float speed;
    float torque;
    uint8_t temperature;
    uint8_t error_code;
    bool valid;
} ak_motor_feedback_t;

typedef struct
{
    float step_speed;
    uint32_t step_interval_ms;
    uint32_t last_step_tick;
    bool step_started;
} ak_motor_step_t;


typedef struct
{
    uint8_t id;
    CAN_HandleTypeDef *hcan;
    float target_speed;
    float command_speed;
    ak_motor_step_t step;
    ak_motor_feedback_t feedback;
} ak_motor_t;

HAL_StatusTypeDef AkMotor_SendServoSpeedCommand(const ak_motor_t *motor);
HAL_StatusTypeDef AkMotor_SendCurrentCommand(const ak_motor_t *motor,
                                             int32_t current);
bool AkMotor_Init(ak_motor_t *motor, CAN_HandleTypeDef *hcan, uint8_t id);
bool AkMotor_SetSpeed(ak_motor_t *motor, float speed);
bool AkMotor_SetCurrent(ak_motor_t *motor, int16_t current);
bool AkMotor_SetSpeedStep(ak_motor_t *motor, float target_rpm,
                          float step_rpm, uint32_t interval_ms);
bool AkMotor_ParseFeedback(ak_motor_t *motor,
                           const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8]);

#endif
