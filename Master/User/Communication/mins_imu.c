/*
 * @file mins_imu.c
 * @brief mins_imu模块的用户接口与实现。
 */

#include "mins_imu.h"

#define MINS_IMU_FRAME_HEAD       0x77U
#define MINS_IMU_RESPONSE_2_14    0x59U
#define MINS_IMU_RESPONSE_2_3     0x84U
#define MINS_IMU_YAW_SCALE        0.973f

<<<<<<< HEAD
imu_t g_imu = {0};

=======
/* 功能：执行 Mins_Imu_CheckFrame 模块操作并维护相关状态。 */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
static uint8_t Mins_Imu_CheckFrame(const uint8_t *frame, uint16_t len,
                                   uint8_t data_len, uint8_t command)
{
    uint16_t i;
    uint8_t checksum = 0U;

    if ((frame == NULL) || (len != (uint16_t)(data_len + 5U)) ||
        (frame[0] != MINS_IMU_FRAME_HEAD) || (frame[1] != (uint8_t)(data_len + 4U)) ||
        (frame[2] != 0U) || (frame[3] != command))
    {
        return 0U;
    }

    for (i = 1U; i < (uint16_t)(len - 1U); i++)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    return (uint8_t)(checksum == frame[len - 1U]);
}

<<<<<<< HEAD
=======
/* 功能：执行 Mins_Imu_DecodeBcd 模块操作并维护相关状态。 */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
static float Mins_Imu_DecodeBcd(const uint8_t *data, uint8_t size, uint8_t decimals)
{
    uint8_t i;
    uint32_t value = (uint32_t)(data[0] & 0x0FU);
    uint32_t divisor = 1U;

    for (i = 1U; i < size; i++)
    {
        value = value * 100U + (uint32_t)((data[i] >> 4U) * 10U) + (data[i] & 0x0FU);
    }
    for (i = 0U; i < decimals; i++)
    {
        divisor *= 10U;
    }

    if ((data[0] & 0xF0U) == 0x10U)
    {
        return -(float)value / (float)divisor;
    }
    return (float)value / (float)divisor;
}

<<<<<<< HEAD
uint8_t Mins_Imu_receive_all(const uint8_t *frame, uint16_t len)
=======
/* 功能：执行 Mins_Imu_receive_all 模块操作并维护相关状态。 */
uint8_t Mins_Imu_receive_all(imu_t *imu, const uint8_t *frame, uint16_t len)
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
{
    uint8_t i;

    if (!Mins_Imu_CheckFrame(frame, len, 43U, MINS_IMU_RESPONSE_2_14))
    {
        return 0U;
    }

    g_imu.pitch = Mins_Imu_DecodeBcd(&frame[4], 3U, 2U);
    g_imu.roll = Mins_Imu_DecodeBcd(&frame[7], 3U, 2U);
    g_imu.yaw = Mins_Imu_DecodeBcd(&frame[10], 3U, 2U) * MINS_IMU_YAW_SCALE;
    for (i = 0U; i < 3U; i++)
    {
        g_imu.accel[i] = Mins_Imu_DecodeBcd(&frame[13U + 3U * i], 3U, 4U);
        g_imu.gyro[i] = Mins_Imu_DecodeBcd(&frame[22U + 3U * i], 3U, 2U);
    }
    for (i = 0U; i < 4U; i++)
    {
        g_imu.quaternion[i] = Mins_Imu_DecodeBcd(&frame[31U + 4U * i], 4U, 6U);
    }

    return 1U;
}

<<<<<<< HEAD
uint8_t Mins_Imu_receive_omega(const uint8_t *frame, uint16_t len)
=======
/* 功能：执行 Mins_Imu_receive_omega 模块操作并维护相关状态。 */
uint8_t Mins_Imu_receive_omega(imu_t *imu, const uint8_t *frame, uint16_t len)
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
{
    if (!Mins_Imu_CheckFrame(frame, len, 9U, MINS_IMU_RESPONSE_2_3))
    {
        return 0U;
    }

    g_imu.pitch = Mins_Imu_DecodeBcd(&frame[4], 3U, 2U);
    g_imu.roll = Mins_Imu_DecodeBcd(&frame[7], 3U, 2U);
    return 1U;
}

<<<<<<< HEAD
=======
/* 功能：执行 Mins_Imu_SetActiveReport 模块操作并维护相关状态。 */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
HAL_StatusTypeDef Mins_Imu_SetActiveReport(UART_HandleTypeDef *uart)
{
    uint8_t command[] = {0x77U, 0x05U, 0x00U, 0x56U, 0x05U, 0x60U};

    if (uart == NULL)
    {
        return HAL_ERROR;
    }
<<<<<<< HEAD
    
=======
/* 功能：执行 HAL_UART_Transmit 模块操作并维护相关状态。 */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
    return HAL_UART_Transmit(uart, command, sizeof(command), 100U);
}