#include "ak_motor.h"
#include "board_can.h"
#include "can.h"
#include "can_app.h"
#include "my_main.h"

static const uint8_t s_motor_ids[4] = {0x03U, 0x04U, 0x01U, 0x02U};
ak_motor_t g_chassis_motors[4];

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

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        /* 5ms任务 */
    }
    else if (htim->Instance == TIM6)
    {
        switch (target_mode_flag)
        {
            case BOARD_MODE_SPEED:

                for (uint8_t i = 0U; i < 4U; ++i)
                {
                    AkMotor_SetSpeed(&g_chassis_motors[i], target_speed[i]);
                }
                break;

            case BOARD_MODE_CURRENT:

                for (uint8_t i = 0U; i < 4U; ++i)
                {
                    AkMotor_SetCurrent(&g_chassis_motors[i], target_current[i]);
                }
                break;

            default:
                break;
        }
        /* 2ms任务 */
    }
    else if (htim->Instance == TIM7)
    {
        /* 1ms任务 */
    }
}
