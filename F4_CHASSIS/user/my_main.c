#include "ak_motor.h"
#include "board_can.h"
#include "can.h"
#include "can_app.h"
#include "my_main.h"

static const uint8_t s_motor_ids[4] = {0x03U, 0x04U, 0x01U, 0x02U};
static ak_motor_t s_chassis_motors[4];

void My_Init(void)
{
    CAN1_Start();
    CAN2_Start();

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (!AkMotor_Init(&s_chassis_motors[i], &hcan2, s_motor_ids[i])
            || !AkMotor_SetSpeed(&s_chassis_motors[i], 0.0f))
        {
            Error_Handler();
        }
        HAL_Delay(5U);
    }
}

void My_Loop(void)
{
    board_can_command_t command;

    if (!Board_CAN_TakeCommand(&command))
    {
        return;
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        bool sent;

        if (command.type == BOARD_CAN_COMMAND_SPEED)
        {
            sent = AkMotor_SetSpeed(&s_chassis_motors[i],
                                    (float)command.values[i]);
        }
        else if (command.type == BOARD_CAN_COMMAND_CURRENT)
        {
            sent = AkMotor_SetCurrent(&s_chassis_motors[i],
                                      command.values[i]);
        }
        else
        {
            return;
        }

        if (!sent)
        {
            Error_Handler();
        }
        HAL_Delay(1U);
    }
}
