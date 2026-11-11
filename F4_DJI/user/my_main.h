#ifndef MY_MAIN_H
#define MY_MAIN_H

#include <stdint.h>
#include "bsp_can.h"

#define MY_CAN1_DJI_CONTROL_ID 0x303U

typedef struct
{
    uint8_t lift_mode;
    uint8_t step_mode;
    float target_speed;
    uint8_t valid;
} my_can1_dji_command_t;

extern volatile my_can1_dji_command_t my_can1_dji_command;

void My_CAN1_Init(void);
void My_CAN1_OnFrame(bsp_can_t *self, BSP_CAN_Frame_t *frame);

#endif
