#ifndef CAN_BOARD_H
#define CAN_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_BOARD_CHASSIS_SPEED_ID   0x301U
#define CAN_BOARD_CHASSIS_CURRENT_ID 0x302U
#define CAN_BOARD_DJI_CONTROL_ID     0x303U

void CAN_Board_Init(void);
bool CAN_Board_SendChassisSpeed(const int16_t values[4]);
bool CAN_Board_SendChassisCurrent(const int16_t values[4]);
bool CAN_Board_SendDjiControl(uint8_t lift_mode, uint8_t step_mode,
                              float target_speed);

#endif
