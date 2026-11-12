#include "ak_motor.h"
#include "board_can.h"
#include "can.h"
#include "chassis_kinematics.h"
#include "can_app.h"
#include "my_main.h"

static const uint8_t s_motor_ids[4] = {0x03U, 0x04U, 0x01U, 0x02U};
ak_motor_t g_chassis_motors[4];
float motor_speed[4] = {0};
float motor_torque[4] = {0};
chassis_speed_t chassis_target_speed = {0};
chassis_speed_t chassis_actual_speed = {0};
chassis_force_t chassis_target_force = {0};
uint8_t a = 0;

void My_Init(void)
{
    while (CAN1_init() != HAL_OK){};

    while (CAN2_init() != HAL_OK){};

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        AkMotor_Init(&g_chassis_motors[i], &hcan2, s_motor_ids[i]);
    }

}

void My_Loop(void)
{
    static uint32_t last_send_tick = 0U;

    /* 2ms 发送一次 */
    if (HAL_GetTick() - last_send_tick < 2U)
    {
        return;
    }

    last_send_tick = HAL_GetTick();

    switch (target_mode_flag)
    {
        case BOARD_MODE_SPEED:

            /* 计算四个电机目标速度 */
            Chassis_InverseKinematics(&chassis_target_speed, motor_speed);

            /* 发送四个电机速度命令 */
            for (uint8_t i = 0U; i < 4U; i++)
            {
                AkMotor_SetSpeed(&g_chassis_motors[i], motor_speed[i]);
            }
            break;

        case BOARD_MODE_CURRENT:

            /* 计算四个电机目标扭矩 */
            Chassis_ForceInverseKinematics(&chassis_target_force, motor_torque);

            /* 发送四个电机电流命令 */
            for (uint8_t i = 0U; i < 4U; i++)
            {
                AkMotor_SetCurrent(&g_chassis_motors[i],
                                   motor_torque[i] * torque_to_current);
            }
            break;

        default:
            break;
    }
}

//1 - rr 03
//2 + lf 04
//3 + lr 01
//0 _ rf 02
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)     //2ms
    {
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            motor_speed[i] = g_chassis_motors[i].feedback.speed;
        }

        /* 解算底盘实际速度 */
        Chassis_ForwardKinematics(motor_speed, &chassis_actual_speed);
    }
}