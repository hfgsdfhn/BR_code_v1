#include "can_board.h"
#include "bsp_can.h"
#include "can.h"
#include <stddef.h>
#include <string.h>

static bsp_can_t s_can_board = {0};

void CAN_Board_Init(void)
{
    s_can_board.handle = &hcan1;
    BSP_CAN_Init(&s_can_board, std);
    BSP_CAN_Bind(&s_can_board);
}

static bool CAN_Board_SendInt16x4(uint32_t id, const int16_t values[4])
{
    BSP_CAN_Frame_t frame = {0};

    if (values == NULL) return false;

    frame.id = id;
    frame.ext = 0U;
    frame.dlc = 8U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        uint16_t value = (uint16_t)values[i];
        frame.data[2U * i] = (uint8_t)value;
        frame.data[2U * i + 1U] = (uint8_t)(value >> 8);
    }

    return BSP_CAN_Send(&s_can_board, &frame);
}

bool CAN_Board_SendChassisSpeed(const int16_t values[4])
{
    return CAN_Board_SendInt16x4(CAN_BOARD_CHASSIS_SPEED_ID, values);
}

bool CAN_Board_SendChassisCurrent(const int16_t values[4])
{
    return CAN_Board_SendInt16x4(CAN_BOARD_CHASSIS_CURRENT_ID, values);
}

bool CAN_Board_SendDjiControl(uint8_t lift_mode, uint8_t step_mode,
                              float target_speed)
{
    BSP_CAN_Frame_t frame = {0};
    uint32_t speed_bits;

    frame.id = CAN_BOARD_DJI_CONTROL_ID;
    frame.ext = 0U;
    frame.dlc = 6U;
    frame.data[0] = lift_mode;
    frame.data[1] = step_mode;
    memcpy(&speed_bits, &target_speed, sizeof(speed_bits));
    frame.data[2] = (uint8_t)speed_bits;
    frame.data[3] = (uint8_t)(speed_bits >> 8);
    frame.data[4] = (uint8_t)(speed_bits >> 16);
    frame.data[5] = (uint8_t)(speed_bits >> 24);

    return BSP_CAN_Send(&s_can_board, &frame);
}
