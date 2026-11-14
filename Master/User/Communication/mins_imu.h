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
    float yaw;
    float pitch;
    float roll;
    float accel[3];
    float gyro[3];
    float quaternion[4];
} imu_t;

/* 功能：接收并解析Mins_Imu_receive_all对应的输入数据，更新模块状态。 */
uint8_t Mins_Imu_receive_all(imu_t *imu, const uint8_t *frame, uint16_t len);
/* 功能：接收并解析Mins_Imu_receive_omega对应的输入数据，更新模块状态。 */
uint8_t Mins_Imu_receive_omega(imu_t *imu, const uint8_t *frame, uint16_t len);
/* 功能：处理Mins_Imu_SetActiveReport对应的输入并更新相关状态。 */
HAL_StatusTypeDef Mins_Imu_SetActiveReport(UART_HandleTypeDef *uart);

#endif