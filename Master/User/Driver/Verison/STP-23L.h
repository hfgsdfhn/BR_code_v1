/*
 * @file STP-23L.h
 * @brief STP-23L模块的用户接口与实现。
 */

#ifndef STP23L_H
#define STP23L_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define STP23L_HEADER_BYTE 0xAAU
#define STP23L_DEVICE_ADDRESS 0x00U
#define STP23L_DISTANCE_COMMAND 0x02U
#define STP23L_HEADER_SIZE 4U
#define STP23L_POINT_SIZE 15U
#define STP23L_DATA_OFFSET 10U
#define STP23L_TIMESTAMP_OFFSET 190U
#define STP23L_CHECKSUM_OFFSET 194U
#define STP23L_BAUD_RATE 230400U
#define STP23L_POINT_COUNT 12U
#define STP23L_FRAME_SIZE 195U
#define STP23L_DATA_SIZE 184U

    typedef struct
    {
        int16_t distance_mm; /* 成员变量。 */
        uint16_t noise; /* 成员变量。 */
        uint32_t peak; /* 成员变量。 */
        uint8_t confidence; /* 成员变量。 */
        uint32_t integration_count; /* 数据长度或数量。 */
        int16_t temperature_reference; /* 成员变量。 */
    } STP23L_Point;

    typedef struct
    {
        STP23L_Point points[STP23L_POINT_COUNT]; /* 成员变量。 */
        uint32_t sensor_timestamp; /* 成员变量。 */
        uint32_t received_tick; /* 成员变量。 */
        uint32_t frame_number; /* 数据缓冲区或数据帧。 */
    } STP23L_Frame;

    typedef struct
    {
        uint32_t valid_frame_count; /* 数据缓冲区或数据帧。 */
        uint32_t checksum_error_count; /* 数据长度或数量。 */
        uint32_t format_error_count; /* 数据长度或数量。 */
        uint32_t uart_error_count; /* 数据长度或数量。 */
        uint32_t receive_restart_error_count; /* 数据长度或数量。 */
        uint32_t last_uart_error; /* 成员变量。 */
        uint32_t last_frame_tick; /* 数据缓冲区或数据帧。 */
    } STP23L_Stats;


/* 功能：执行 STP23L_Init 函数对应的模块操作。 */
    HAL_StatusTypeDef STP23L_Init(UART_HandleTypeDef *uart_handle);


/* 功能：执行 STP23L_HandleUartRxComplete 函数对应的模块操作。 */
    void STP23L_HandleUartRxComplete(UART_HandleTypeDef *uart_handle);

/* 功能：执行 STP23L_HandleUartError 函数对应的模块操作。 */
    void STP23L_HandleUartError(UART_HandleTypeDef *uart_handle);


/* 功能：执行 STP23L_GetLatestFrame 函数对应的模块操作。 */
    uint8_t STP23L_GetLatestFrame(STP23L_Frame *frame);

/* 功能：执行 STP23L_GetAverageDistance 函数对应的模块操作。 */
    uint8_t STP23L_GetAverageDistance(uint8_t minimum_confidence, uint16_t *distance_mm);

/* 功能：执行 STP23L_IsOnline 函数对应的模块操作。 */
    uint8_t STP23L_IsOnline(uint32_t timeout_ms);

/* 功能：执行 STP23L_GetStats 函数对应的模块操作。 */
    void STP23L_GetStats(STP23L_Stats *stats);

#ifdef __cplusplus
}
#endif

#endif
