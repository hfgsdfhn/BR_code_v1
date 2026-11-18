
#ifndef MINS_IMU_H
#define MINS_IMU_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float accel[3];
    float gyro[3];
    float quaternion[4];
} imu_t;

extern imu_t g_imu;

uint8_t Mins_Imu_receive_all(const uint8_t *frame, uint16_t len);
void Mins_Imu_receive_omega(const uint8_t *frame, uint16_t len);
HAL_StatusTypeDef Mins_Imu_SetActiveReport(UART_HandleTypeDef *uart);

#endif
