#include "STP-23L.h"

#include <stddef.h>

static UART_HandleTypeDef *stp23l_uart;
static uint8_t stp23l_rx_byte;
static uint8_t stp23l_rx_frame[STP23L_FRAME_SIZE];
static uint16_t stp23l_rx_index;
static STP23L_Frame stp23l_latest_frame;
static STP23L_Stats stp23l_stats;

static void STP23L_ParseByte(uint8_t data);
static uint8_t STP23L_IsValidFrame(void);
static void STP23L_DecodeFrame(void);
static void STP23L_KeepTrailingHeader(void);
static void STP23L_ArmReceive(void);
static uint16_t STP23L_ReadU16(const uint8_t *data);
static uint32_t STP23L_ReadU32(const uint8_t *data);
static uint32_t STP23L_EnterCritical(void);
static void STP23L_ExitCritical(uint32_t primask);

HAL_StatusTypeDef STP23L_Init(UART_HandleTypeDef *uart_handle)
{
    uint32_t primask;

    if (uart_handle == NULL || uart_handle->Init.BaudRate != STP23L_BAUD_RATE ||
        uart_handle->Init.WordLength != UART_WORDLENGTH_8B || uart_handle->Init.StopBits != UART_STOPBITS_1 ||
        uart_handle->Init.Parity != UART_PARITY_NONE)
    {
        return HAL_ERROR;
    }

    primask = STP23L_EnterCritical();
    stp23l_uart = uart_handle;
    stp23l_rx_index = 0U;
    stp23l_latest_frame.frame_number = 0U;
    stp23l_stats.valid_frame_count = 0U;
    stp23l_stats.checksum_error_count = 0U;
    stp23l_stats.format_error_count = 0U;
    stp23l_stats.uart_error_count = 0U;
    stp23l_stats.receive_restart_error_count = 0U;
    stp23l_stats.last_uart_error = HAL_UART_ERROR_NONE;
    stp23l_stats.last_frame_tick = 0U;
    STP23L_ExitCritical(primask);

    return HAL_UART_Receive_DMA(stp23l_uart, &stp23l_rx_byte, 1U);
}

void STP23L_HandleUartRxComplete(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL || uart_handle != stp23l_uart)
    {
        return;
    }

    STP23L_ParseByte(stp23l_rx_byte);
    STP23L_ArmReceive();
}

void STP23L_HandleUartError(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL || uart_handle != stp23l_uart)
    {
        return;
    }

    ++stp23l_stats.uart_error_count;
    stp23l_stats.last_uart_error = uart_handle->ErrorCode;
    stp23l_rx_index = 0U;
    STP23L_ArmReceive();
}

uint8_t STP23L_GetLatestFrame(STP23L_Frame *frame)
{
    uint32_t primask;

    if (frame == NULL)
    {
        return 0U;
    }

    primask = STP23L_EnterCritical();
    if (stp23l_stats.valid_frame_count == 0U)
    {
        STP23L_ExitCritical(primask);
        return 0U;
    }
    *frame = stp23l_latest_frame;
    STP23L_ExitCritical(primask);
    return 1U;
}

uint8_t STP23L_GetAverageDistance(uint8_t minimum_confidence, uint16_t *distance_mm)
{
    STP23L_Frame frame;
    uint32_t sum;
    uint32_t count;
    uint32_t index;

    if (distance_mm == NULL || STP23L_GetLatestFrame(&frame) == 0U)
    {
        return 0U;
    }

    sum = 0U;
    count = 0U;
    for (index = 0U; index < STP23L_POINT_COUNT; ++index)
    {
        if (frame.points[index].distance_mm > 0 && frame.points[index].confidence >= minimum_confidence)
        {
            sum += (uint16_t)frame.points[index].distance_mm;
            ++count;
        }
    }

    if (count == 0U)
    {
        return 0U;
    }

    *distance_mm = (uint16_t)(sum / count);
    return 1U;
}

uint8_t STP23L_IsOnline(uint32_t timeout_ms)
{
    uint32_t last_frame_tick;
    uint32_t valid_frame_count;
    uint32_t primask;

    primask = STP23L_EnterCritical();
    last_frame_tick = stp23l_stats.last_frame_tick;
    valid_frame_count = stp23l_stats.valid_frame_count;
    STP23L_ExitCritical(primask);

    return (uint8_t)(valid_frame_count != 0U && (uint32_t)(HAL_GetTick() - last_frame_tick) <= timeout_ms);
}

void STP23L_GetStats(STP23L_Stats *stats)
{
    uint32_t primask;

    if (stats == NULL)
    {
        return;
    }

    primask = STP23L_EnterCritical();
    *stats = stp23l_stats;
    STP23L_ExitCritical(primask);
}

static void STP23L_ParseByte(uint8_t data)
{
    if (stp23l_rx_index < STP23L_HEADER_SIZE)
    {
        if (data == STP23L_HEADER_BYTE)
        {
            stp23l_rx_frame[stp23l_rx_index] = data;
            ++stp23l_rx_index;
        }
        else
        {
            stp23l_rx_index = 0U;
        }
        return;
    }

    stp23l_rx_frame[stp23l_rx_index] = data;
    ++stp23l_rx_index;
    if (stp23l_rx_index < STP23L_FRAME_SIZE)
    {
        return;
    }

    if (STP23L_IsValidFrame() != 0U)
    {
        STP23L_DecodeFrame();
        stp23l_rx_index = 0U;
    }
    else
    {
        STP23L_KeepTrailingHeader();
    }
}

static uint8_t STP23L_IsValidFrame(void)
{
    uint8_t checksum;
    uint16_t index;

    if (stp23l_rx_frame[4] != STP23L_DEVICE_ADDRESS || stp23l_rx_frame[5] != STP23L_DISTANCE_COMMAND ||
        STP23L_ReadU16(&stp23l_rx_frame[6]) != 0U || STP23L_ReadU16(&stp23l_rx_frame[8]) != STP23L_DATA_SIZE)
    {
        ++stp23l_stats.format_error_count;
        return 0U;
    }

    checksum = 0U;
    for (index = 4U; index < STP23L_CHECKSUM_OFFSET; ++index)
    {
        checksum = (uint8_t)(checksum + stp23l_rx_frame[index]);
    }
    if (checksum != stp23l_rx_frame[STP23L_CHECKSUM_OFFSET])
    {
        ++stp23l_stats.checksum_error_count;
        return 0U;
    }

    return 1U;
}

static void STP23L_DecodeFrame(void)
{
    uint32_t index;
    uint32_t offset;
    uint32_t now;

    for (index = 0U; index < STP23L_POINT_COUNT; ++index)
    {
        offset = STP23L_DATA_OFFSET + index * STP23L_POINT_SIZE;
        stp23l_latest_frame.points[index].distance_mm = (int16_t)STP23L_ReadU16(&stp23l_rx_frame[offset]);
        stp23l_latest_frame.points[index].noise = STP23L_ReadU16(&stp23l_rx_frame[offset + 2U]);
        stp23l_latest_frame.points[index].peak = STP23L_ReadU32(&stp23l_rx_frame[offset + 4U]);
        stp23l_latest_frame.points[index].confidence = stp23l_rx_frame[offset + 8U];
        stp23l_latest_frame.points[index].integration_count = STP23L_ReadU32(&stp23l_rx_frame[offset + 9U]);
        stp23l_latest_frame.points[index].temperature_reference =
            (int16_t)STP23L_ReadU16(&stp23l_rx_frame[offset + 13U]);
    }

    now = HAL_GetTick();
    stp23l_latest_frame.sensor_timestamp = STP23L_ReadU32(&stp23l_rx_frame[STP23L_TIMESTAMP_OFFSET]);
    stp23l_latest_frame.received_tick = now;
    ++stp23l_stats.valid_frame_count;
    stp23l_stats.last_frame_tick = now;
    stp23l_latest_frame.frame_number = stp23l_stats.valid_frame_count;
}

static void STP23L_KeepTrailingHeader(void)
{
    uint16_t trailing_count;
    uint16_t index;

    trailing_count = 0U;
    index = STP23L_FRAME_SIZE;
    while (index > 0U && trailing_count < STP23L_HEADER_SIZE && stp23l_rx_frame[index - 1U] == STP23L_HEADER_BYTE)
    {
        --index;
        ++trailing_count;
    }

    for (index = 0U; index < trailing_count; ++index)
    {
        stp23l_rx_frame[index] = STP23L_HEADER_BYTE;
    }
    stp23l_rx_index = trailing_count;
}

static void STP23L_ArmReceive(void)
{
    if (HAL_UART_Receive_DMA(stp23l_uart, &stp23l_rx_byte, 1U) != HAL_OK)
    {
        ++stp23l_stats.receive_restart_error_count;
    }
}

static uint16_t STP23L_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t STP23L_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static uint32_t STP23L_EnterCritical(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void STP23L_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

