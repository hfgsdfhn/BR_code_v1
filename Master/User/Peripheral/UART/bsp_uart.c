/*
 * @file bsp_uart.c
 * @brief bsp_uart模块的用户接口与实现。
 */

#include "bsp_uart.h"
#include <string.h>
#include "cmsis_os2.h"
#include "mins_imu.h"
#include "remote.h"
#include "usart.h"
#include "STP-23L.h"

__ALIGNED(CACHE_LINE_SIZE) static uint8_t imu_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) static uint8_t dt35_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) static uint8_t ustp23l_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) static uint8_t dstp23l_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) static uint8_t scram_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) static uint8_t remote_rx[UART_RX_BUFFER_SIZE];

static uint8_t BspUart_ImuRxCallback(const uint8_t *data, uint16_t size)
{
    return Mins_Imu_receive_omega(data, size);
}

bsp_uart_t uart_list[BSP_UART_COUNT] = {
    [BSP_UART_IMU] = {.handle = &huart3, .buffer = imu_rx, .callback = BspUart_ImuRxCallback},
    [BSP_UART_DT35] = {.handle = &huart4, .buffer = dt35_rx},
    [BSP_UART_UP_STP23L] = {.handle = &huart5, .buffer = ustp23l_rx},
    [BSP_UART_DOWN_STP23L] = {.handle = &huart6, .buffer = dstp23l_rx},
    [BSP_UART_SCRAM] = {.handle = &huart7, .buffer = scram_rx},
    [BSP_UART_REMOTE] = {.handle = &huart9, .buffer = remote_rx, .callback = Remote_Receive}
};

static osThreadId_t uart_task_handle;

/* 功能：执行 BspUart_RegisterRxCallback 函数对应的模块操作。 */
bool BspUart_RegisterRxCallback(bsp_uart_channel_t channel, bsp_uart_rx_callback_t callback)
{
    if ((uint32_t)channel >= BSP_UART_COUNT) return false;
    uart_list[channel].callback = callback;
    return true;
}

/* 功能：执行 BspUart_GetAlignedCacheRange 函数对应的模块操作。 */
static void BspUart_GetAlignedCacheRange(const uint8_t *buffer, uint32_t size,
                                         uint8_t **aligned_buffer, uint32_t *aligned_size)
{
    uint32_t address = (uint32_t)buffer;
    uint32_t start = address & ~(CACHE_LINE_SIZE - 1U);
    uint32_t offset = address - start;
    *aligned_buffer = (uint8_t *)start;
    *aligned_size = (size + offset + CACHE_LINE_SIZE - 1U) & ~(CACHE_LINE_SIZE - 1U);
}

/* 功能：执行 BspUart_Invalidate 函数对应的模块操作。 */
static void BspUart_Invalidate(uint8_t *buffer)
{
    uint8_t *aligned_buffer;
    uint32_t aligned_size;
    BspUart_GetAlignedCacheRange(buffer, UART_RX_BUFFER_SIZE, &aligned_buffer, &aligned_size);
    SCB_InvalidateDCache_by_Addr(aligned_buffer, (int32_t)aligned_size);
}

/* 功能：执行 BspUart_StartReceive 函数对应的模块操作。 */
static HAL_StatusTypeDef BspUart_StartReceive(bsp_uart_t *uart)
{
    HAL_StatusTypeDef status;
    if ((uart == NULL) || (uart->handle == NULL) || (uart->handle->hdmarx == NULL)) return HAL_ERROR;

    CLEAR_BIT(uart->handle->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_SEND_REQ(uart->handle, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(uart->handle,
                          UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF |
                          UART_CLEAR_OREF | UART_CLEAR_IDLEF);
    uart->handle->ErrorCode = HAL_UART_ERROR_NONE;
    uart->position = 0U;

    {
        uint8_t *aligned_buffer;
        uint32_t aligned_size;
        BspUart_GetAlignedCacheRange(uart->buffer, UART_RX_BUFFER_SIZE,
                                      &aligned_buffer, &aligned_size);
        SCB_CleanInvalidateDCache_by_Addr(aligned_buffer, (int32_t)aligned_size);
    }

    status = HAL_UART_Receive_DMA(uart->handle, uart->buffer, UART_RX_BUFFER_SIZE);
    if (status == HAL_OK) __HAL_DMA_DISABLE_IT(uart->handle->hdmarx, DMA_IT_HT);
    return status;
}

/* 功能：执行 BspUart_RequestRestart 函数对应的模块操作。 */
static void BspUart_RequestRestart(uint32_t index)
{
    uart_list[index].restart_requested = true;
    if (uart_task_handle != NULL) (void)osThreadFlagsSet(uart_task_handle, UART_RESTART_FLAG(index));
}

/* 功能：执行 BspUart_Init 函数对应的模块操作。 */
void BspUart_Init(void)
{
    for (uint32_t index = 0U; index < BSP_UART_COUNT; index++)
    {
        bsp_uart_t *uart = &uart_list[index];
        uart->restart_requested = false;
        if (uart->callback != NULL && BspUart_StartReceive(uart) != HAL_OK)
            BspUart_RequestRestart(index);
    }
}

/* 功能：执行 BspUart_Task 函数对应的模块操作。 */
void BspUart_Task(void *argument)
{
    (void)argument;
    uart_task_handle = osThreadGetId();
    BspUart_Init();

    for (;;)
    {
        for (uint32_t index = 0U; index < BSP_UART_COUNT; index++)
        {
            bsp_uart_t *uart = &uart_list[index];
            if (uart->handle == NULL || uart->handle->hdmarx == NULL) continue;

            if (uart->restart_requested)
            {
                uart->restart_requested = false;
                (void)HAL_UART_AbortReceive(uart->handle);
                if (BspUart_StartReceive(uart) != HAL_OK) uart->restart_requested = true;
                if (index == BSP_UART_REMOTE) Remote_UartError();
                continue;
            }

            BspUart_Invalidate(uart->buffer);
            uint16_t write = (uint16_t)(UART_RX_BUFFER_SIZE -
                                        __HAL_DMA_GET_COUNTER(uart->handle->hdmarx));
            if (write != uart->position && uart->callback != NULL)
            {
                if (write > uart->position)
                {
                    uart->callback(&uart->buffer[uart->position],
                                   (uint16_t)(write - uart->position));
                }
                else
                {
                    uart->callback(&uart->buffer[uart->position],
                                   (uint16_t)(UART_RX_BUFFER_SIZE - uart->position));
                    if (write != 0U) uart->callback(uart->buffer, write);
                }
            }
            uart->position = write;
        }
        osDelay(1U);
    }
}

/* 功能：执行 BspUart_Send 函数对应的模块操作。 */
bool BspUart_Send(bsp_uart_channel_t channel, const uint8_t *data, uint16_t size)
{
    if ((channel >= BSP_UART_COUNT) || (data == NULL) || (size == 0U) ||
        (size > UART_TX_MAX_SIZE)) return false;

    bsp_uart_t *uart = &uart_list[channel];
    if (uart->handle == NULL) return false;

    bool start = false;
    __disable_irq();
    if (uart->tx.count >= UART_TX_QUEUE_DEPTH)
    {
        __enable_irq();
        return false;
    }
    uint8_t write = uart->tx.write;
    memcpy(uart->tx.buffer[write], data, size);
    uart->tx.size[write] = size;
    uart->tx.write = (uint8_t)((write + 1U) % UART_TX_QUEUE_DEPTH);
    uart->tx.count++;
    if (!uart->tx.active)
    {
        uart->tx.active = true;
        start = true;
    }
    __enable_irq();

    if (start && HAL_UART_Transmit_DMA(uart->handle, uart->tx.buffer[uart->tx.read],
                                       uart->tx.size[uart->tx.read]) != HAL_OK)
    {
        __disable_irq();
        uart->tx.read = uart->tx.write;
        uart->tx.count = 0U;
        uart->tx.active = false;
        __enable_irq();
        return false;
    }
    return true;
}

/* 功能：执行 HAL_UART_TxCpltCallback 函数对应的模块操作。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint32_t index = 0U; index < BSP_UART_COUNT; index++)
    {
        bsp_uart_t *uart = &uart_list[index];
        if (uart->handle != huart) continue;
        if (uart->tx.count == 0U)
        {
            uart->tx.active = false;
            return;
        }
        uart->tx.read = (uint8_t)((uart->tx.read + 1U) % UART_TX_QUEUE_DEPTH);
        uart->tx.count--;
        if (uart->tx.count == 0U)
        {
            uart->tx.active = false;
            return;
        }
        if (HAL_UART_Transmit_DMA(uart->handle, uart->tx.buffer[uart->tx.read],
                                  uart->tx.size[uart->tx.read]) != HAL_OK)
        {
            uart->tx.read = uart->tx.write;
            uart->tx.count = 0U;
            uart->tx.active = false;
        }
        return;
    }
}

/* 功能：执行 HAL_UART_ErrorCallback 函数对应的模块操作。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        STP23L_HandleUartError(huart);
        return;
    }
    for (uint32_t index = 0U; index < BSP_UART_COUNT; index++)
    {
        if (huart == uart_list[index].handle)
        {
            if (index == BSP_UART_REMOTE) Remote_UartError();
            BspUart_RequestRestart(index);
            return;
        }
    }
}

/* 功能：执行 HAL_UARTEx_RxEventCallback 函数对应的模块操作。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    (void)size;
}
