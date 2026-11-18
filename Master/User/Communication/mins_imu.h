#ifndef MINS_IMU_H
#define MINS_IMU_H

#include <stdint.h>

#include "bsp_uart.h"


typedef struct
{
    float pitch;
    float roll;
    float yaw;
} mins_imu_euler_t;

typedef struct
{
    bsp_uart_t *uart;
    mins_imu_euler_t euler;
    float yaw_last_raw;
    float yaw_total;
    uint8_t yaw_last_valid;
    uint32_t update_count;
} mins_imu_t;

void Mins_Imu_Init(mins_imu_t *imu, bsp_uart_t *uart);
void Mins_Imu_RequestEuler(mins_imu_t *imu);

void Mins_Imu_OnRx(const uint8_t *data, uint16_t len);

extern mins_imu_t g_imu;


#endif /* MINS_IMU_H */
