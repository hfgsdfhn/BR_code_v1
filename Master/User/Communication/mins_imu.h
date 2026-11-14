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

/* 提前定义全局结构体变量，供外部直接引用读取数据 */
extern imu_t g_imu;

/* 功能：执行 Mins_Imu_receive_all 函数对应的模块操作。 */
uint8_t Mins_Imu_receive_all(const uint8_t *frame, uint16_t len);
/* 功能：执行 Mins_Imu_receive_omega 函数对应的模块操作。 */
uint8_t Mins_Imu_receive_omega(const uint8_t *frame, uint16_t len);
/* 功能：执行 Mins_Imu_SetActiveReport 函数对应的模块操作。 */
HAL_StatusTypeDef Mins_Imu_SetActiveReport(UART_HandleTypeDef *uart);

#endif