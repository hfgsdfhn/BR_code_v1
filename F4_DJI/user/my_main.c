#include "my_main.h"
#include "can.h"

#include <string.h>

volatile my_can1_dji_command_t my_can1_dji_command;
static bsp_can_t s_can1 = {0};

void My_CAN1_Init(void)
{
    s_can1.handle = &hcan1;
    s_can1.on_rx = My_CAN1_OnFrame;
    BSP_CAN_Init(&s_can1, std);
    BSP_CAN_Bind(&s_can1);
}

void My_CAN1_OnFrame(bsp_can_t *self, BSP_CAN_Frame_t *frame)
{
    uint32_t speed_bits;
    float target_speed;

    if (self == NULL || frame == NULL || frame->ext != 0U ||
        frame->id != MY_CAN1_DJI_CONTROL_ID || frame->dlc != 6U)
    {
        return;
    }

    my_can1_dji_command.lift_mode = frame->data[0];
    my_can1_dji_command.step_mode = frame->data[1];
    speed_bits = (uint32_t)frame->data[2] |
                 ((uint32_t)frame->data[3] << 8U) |
                 ((uint32_t)frame->data[4] << 16U) |
                 ((uint32_t)frame->data[5] << 24U);
    memcpy(&target_speed, &speed_bits, sizeof(speed_bits));
    my_can1_dji_command.target_speed = target_speed;
    my_can1_dji_command.valid = 1U;
}
