#include "my_main.h"
#include "ak_motor.h"
#include "board_can.h"
#include "can.h"
#include "chassis_kinematics.h"
#include "can_app.h"

volatile uint8_t target_mode = 0;
static const uint8_t s_motor_ids[4] = {0x03U, 0x04U, 0x01U, 0x02U};
ak_motor_t g_chassis_motors[4];
float feedback_motor_speed[4] = {0};
float target_motor_speed[4]   = {0};
float target_motor_torque[4]  = {0};

/* 底盘目标/实际量 */
chassis_speed_t chassis_target_speed;
chassis_speed_t chassis_actual_speed;
chassis_force_t chassis_target_force;


void My_Init(void)
{
    CAN1_init();
    CAN2_init();

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        AkMotor_Init(&g_chassis_motors[i], &hcan2, s_motor_ids[i]);
    }
}


void My_Loop(void)
{
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) { return; }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        feedback_motor_speed[i] = g_chassis_motors[i].feedback.speed;
    }
    Chassis_ForwardKinematics(feedback_motor_speed, &chassis_actual_speed);

    if (target_mode == 0U)
    {
        /* ---- 速度模式 ---- */
        Chassis_InverseKinematics(&chassis_target_speed, target_motor_speed);
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            AkMotor_SetSpeed(&g_chassis_motors[i], target_motor_speed[i]);
        }
    }
    else
    {
        /* ---- 电流模式 ---- */
        Chassis_ForceInverseKinematics(&chassis_target_force, target_motor_torque);
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            AkMotor_SetCurrent(&g_chassis_motors[i],
                               target_motor_torque[i] * torque_to_current);
        }
    }
}