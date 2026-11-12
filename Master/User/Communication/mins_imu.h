/*
 * @file mins_imu.h
 * @brief mins_imu模块的用户接口与实现。
 */

#ifndef MINS_IMU_H
#define MINS_IMU_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef struct
{
    float yaw; /* 成员变量。 */
    float pitch; /* 成员变量。 */
    float roll; /* 成员变量。 */
    float accel[3]; /* 成员变量。 */
    float gyro[3]; /* 成员变量。 */
    float quaternion[4]; /* 成员变量。 */
} imu_t;

/* 功能：执行 Mins_Imu_receive_all 函数对应的模块操作。 */
uint8_t Mins_Imu_receive_all(imu_t *imu, const uint8_t *frame, uint16_t len);
/* 功能：执行 Mins_Imu_receive_omega 函数对应的模块操作。 */
uint8_t Mins_Imu_receive_omega(imu_t *imu, const uint8_t *frame, uint16_t len);
/* 功能：执行 Mins_Imu_SetActiveReport 函数对应的模块操作。 */
HAL_StatusTypeDef Mins_Imu_SetActiveReport(UART_HandleTypeDef *uart);

#endif