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
        int16_t distance_mm;
        uint16_t noise;
        uint32_t peak;
        uint8_t confidence;
        uint32_t integration_count;
        int16_t temperature_reference;
    } STP23L_Point;

    typedef struct
    {
        STP23L_Point points[STP23L_POINT_COUNT];
        uint32_t sensor_timestamp;
        uint32_t received_tick;
        uint32_t frame_number;
    } STP23L_Frame;

    typedef struct
    {
        uint32_t valid_frame_count;
        uint32_t checksum_error_count;
        uint32_t format_error_count;
        uint32_t uart_error_count;
        uint32_t receive_restart_error_count;
        uint32_t last_uart_error;
        uint32_t last_frame_tick;
    } STP23L_Stats;

    /* 所选 UART 必须已初始化为 230400 波特率、8 位数据位、无校验、1 位停止位。 */
    HAL_StatusTypeDef STP23L_Init(UART_HandleTypeDef *uart_handle);

    /* 将对应的 HAL UART 回调转发至这两个函数。 */
    void STP23L_HandleUartRxComplete(UART_HandleTypeDef *uart_handle);
    void STP23L_HandleUartError(UART_HandleTypeDef *uart_handle);

    uint8_t STP23L_GetLatestFrame(STP23L_Frame *frame);
    uint8_t STP23L_GetAverageDistance(uint8_t minimum_confidence, uint16_t *distance_mm);
    uint8_t STP23L_IsOnline(uint32_t timeout_ms);
    void STP23L_GetStats(STP23L_Stats *stats);

#ifdef __cplusplus
}
#endif

#endif /* 头文件保护结束。 */
