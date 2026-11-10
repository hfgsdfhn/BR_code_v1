/* UART DMA receive and nonblocking transmit support. */

#include "bsp_uart.h"

#include <string.h>

#include "cmsis_os2.h"
#include "usart.h"
#include "STP-23L.h"
#include "remote.h"

__ALIGNED(CACHE_LINE_SIZE) uint8_t computer_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) uint8_t imu_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) uint8_t dt35_rx[UART_RX_BUFFER_SIZE];
__ALIGNED(CACHE_LINE_SIZE) uint8_t remote_rx[UART_RX_BUFFER_SIZE];

bsp_uart_t uart_list[BSP_UART_COUNT] = {
    [BSP_UART_COMPUTER] = {&huart2, computer_rx, 0U, NULL},
    [BSP_UART_DT35] = {&huart4, dt35_rx, 0U, NULL},
    [BSP_UART_IMU] = {&huart8, imu_rx, 0U, NULL},
    [BSP_UART_SMALL_BOARD] = {&huart9, remote_rx, 0U, NULL}
};

static osThreadId_t uart_task_handle;

bool BspUart_RegisterRxCallback(bsp_uart_channel_t channel, bsp_uart_rx_callback_t callback)
{
    if ((uint32_t)channel >= BSP_UART_COUNT)
    {
        return false;
    }
    uart_list[channel].callback = callback;
    return true;
}

static void BspUart_GetAlignedCacheRange(const uint8_t *buffer, uint32_t size, uint8_t **aligned_buffer,
                                         uint32_t *aligned_size)
{
    uint32_t address = (uint32_t)buffer;
    uint32_t start = address & ~(CACHE_LINE_SIZE - 1U);
    uint32_t offset = address - start;

    *aligned_buffer = (uint8_t *)start;
    *aligned_size = (size + offset + CACHE_LINE_SIZE - 1U) & ~(CACHE_LINE_SIZE - 1U);
}

/* 使接收缓冲区对应的数据缓存失效，确保 CPU 读取 DMA 写入的最新数据。 */
static void BspUart_Invalidate(uint8_t *buffer /* DMA 接收缓冲区的起始地址。 */)
{
    uint8_t *aligned_buffer;
    uint32_t aligned_size;

    BspUart_GetAlignedCacheRange(buffer, UART_RX_BUFFER_SIZE, &aligned_buffer, &aligned_size);
    SCB_InvalidateDCache_by_Addr(aligned_buffer, (int32_t)aligned_size);
}

static HAL_StatusTypeDef BspUart_StartReceive(bsp_uart_t *uart)
{
    HAL_StatusTypeDef status;

    if ((uart == NULL) || (uart->handle == NULL) || (uart->handle->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    /* 暂停 UART 的 DMA 接收请求，清空残留数据和错误标志，再重新启动接收。 */
    CLEAR_BIT(uart->handle->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_SEND_REQ(uart->handle, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(uart->handle,
                          UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
    uart->handle->ErrorCode = HAL_UART_ERROR_NONE;
    uart->position = 0U;
    {
        uint8_t *aligned_buffer;
        uint32_t aligned_size;

        BspUart_GetAlignedCacheRange(uart->buffer, UART_RX_BUFFER_SIZE, &aligned_buffer, &aligned_size);
        SCB_CleanInvalidateDCache_by_Addr(aligned_buffer, (int32_t)aligned_size);
    }

    status = HAL_UART_Receive_DMA(uart->handle, uart->buffer, UART_RX_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(uart->handle->hdmarx, DMA_IT_HT);
    }
    return status;
}

static void BspUart_RequestRestart(uint32_t index)
{
    uart_list[index].restart_requested = true;
    if (uart_task_handle != NULL)
    {
        (void)osThreadFlagsSet(uart_task_handle, UART_RESTART_FLAG(index));
    }
}

/* 为已注册接收回调的 UART 通道启动 DMA 接收，启动失败时请求重试。 */
void BspUart_Init(void)
{
    uint32_t index;

    for (index = 0U; index < BSP_UART_COUNT; index++)
    {
        bsp_uart_t *uart = &uart_list[index];

        uart->restart_requested = false;
        if (uart->callback == NULL)
        {
            continue;
        }
        if (BspUart_StartReceive(uart) != HAL_OK)
        {
            BspUart_RequestRestart(index);
        }
    }
}

void BspUart_Task(void *argument)
{
    (void)argument;
    uart_task_handle = osThreadGetId();
    BspUart_Init();
    for (;;)
    {
        uint32_t index;
        for (index = 0U; index < BSP_UART_COUNT; index++)
        {
            bsp_uart_t *uart = &uart_list[index];
            uint16_t write;
            if (uart->handle == NULL || uart->handle->hdmarx == NULL) continue;
            BspUart_Invalidate(uart->buffer);
            write = (uint16_t)(UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->handle->hdmarx));
            if (write != uart->position && uart->callback != NULL)
            {
                if (write > uart->position)
                    uart->callback(&uart->buffer[uart->position], (uint16_t)(write - uart->position));
                else
                {
                    uart->callback(&uart->buffer[uart->position], (uint16_t)(UART_RX_BUFFER_SIZE - uart->position));
                    if (write != 0U) uart->callback(uart->buffer, write);
                }
            }
            uart->position = write;
            if (uart->restart_requested)
            {
                uart->restart_requested = false;
                (void)HAL_UART_AbortReceive(uart->handle);
                if (BspUart_StartReceive(uart) != HAL_OK) uart->restart_requested = true;
            }
        }
        osDelay(1U);
    }
}
/* Queue data for asynchronous UART transmission. */
bool BspUart_Send(bsp_uart_channel_t channel, const uint8_t *data, uint16_t size)
{
    bsp_uart_t *uart; /* 目标 UART 通道的状态对象。 */
    uint8_t write;    /* 发送队列中用于写入本帧的槽位索引。 */
    bool start;       /* 队列原先空闲时，置位以启动 HAL DMA 发送。 */

    if ((channel >= BSP_UART_COUNT) || (data == NULL) || (size == 0U))
    {
        return false;
    }
    if (size > UART_TX_MAX_SIZE)
    {
        return false;
    }

    uart = &uart_list[channel];
    if (uart->handle == NULL)
    {
        return false;
    }
    start = false;
    __disable_irq();
    if (uart->tx.count >= UART_TX_QUEUE_DEPTH)
    {
        __enable_irq();
        return false;
    }
    write = uart->tx.write;
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

    if (start &&
        (HAL_UART_Transmit_DMA(uart->handle, uart->tx.buffer[uart->tx.read], uart->tx.size[uart->tx.read]) != HAL_OK))
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

/* 一帧发送完成后将其出队；队列非空时继续发送下一帧，否则清除发送忙标志。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart /* 本次完成发送的 UART 句柄。 */)
{
    uint32_t index; /* 用于查找对应 UART 通道的数组索引。 */

    for (index = 0U; index < BSP_UART_COUNT; index++)
    {
        bsp_uart_t *uart = &uart_list[index]; /* 当前检查的 UART 通道及其发送队列。 */

        if (uart->handle != huart)
        {
            continue;
        }
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
        if (HAL_UART_Transmit_DMA(uart->handle, uart->tx.buffer[uart->tx.read], uart->tx.size[uart->tx.read]) != HAL_OK)
        {
            uart->tx.read = uart->tx.write;
            uart->tx.count = 0U;
            uart->tx.active = false;
        }
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart9)
    {
        Remote_UartError();
        return;
    }

    uint32_t index;

    /* USART2 由 STP-23L 驱动处理接收错误，处理后直接返回。 */
    if (huart == &huart2)
    {
        STP23L_HandleUartError(huart);
        return;
    }

    for (index = 0U; index < BSP_UART_COUNT; index++)
    {
        if (huart != uart_list[index].handle)
        {
            continue;
        }

        BspUart_RequestRestart(index);
        return;
    }
}

/* 保留 HAL 接收事件回调入口；DMA 数据由 UART 任务轮询处理，此处无需处理。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart; (void)size;
}









