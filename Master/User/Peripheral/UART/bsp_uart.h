/*
 * @file bsp_uart.h
 * @brief bsp_uart模块的用户接口与实现。
 */

#ifndef BSP_UART_H
#define BSP_UART_H
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#define UART_TX_QUEUE_DEPTH 4U
#define UART_TX_MAX_SIZE 160U
#define UART_RESTART_FLAG(index) (1UL << (index))
#define UART_RESTART_ALL_FLAGS ((1UL << BSP_UART_COUNT) - 1UL)
#define UART_RX_BUFFER_SIZE 256U
#define CACHE_LINE_SIZE 32U
typedef void (*bsp_uart_rx_callback_t)(const uint8_t *data, uint16_t size);
typedef struct
{
    uint8_t buffer[UART_TX_QUEUE_DEPTH][UART_TX_MAX_SIZE]; /* 发送队列中保存的字节数据。 */
    uint16_t size[UART_TX_QUEUE_DEPTH]; /* 每个队列项的有效数据长度。 */
    volatile uint8_t read; /* 下一个待发送队列项的读取位置。 */
    volatile uint8_t write; /* 下一个可写入队列项的位置。 */
    volatile uint8_t count; /* 当前队列中等待发送的项目数量。 */
    volatile bool active; /* 当前是否正在执行异步发送。 */
} bsp_uart_tx_t;
typedef struct
{
    UART_HandleTypeDef *handle; /* 当前通道对应的 HAL UART 外设句柄。 */
    uint8_t *buffer; /* 当前通道的 DMA 循环接收缓冲区。 */
    volatile uint32_t rx_produced; /* DMA 已经写入的接收数据总位置。 */
    uint32_t rx_consumed; /* 应用层已经处理的接收数据总位置。 */
    uint32_t rx_overruns; /* 接收缓冲区发生覆盖的累计次数。 */
    uint16_t position; /* 上一次处理到的循环缓冲区位置。 */
    bsp_uart_rx_callback_t callback; /* 收到新数据后调用的处理函数。 */
    bsp_uart_tx_t tx; /* 当前通道的异步发送队列。 */
    volatile bool restart_requested; /* 接收异常后请求重新启动 UART。 */
} bsp_uart_t;
typedef enum
{
    BSP_UART_COMPUTER = 0, /* 上位机通信使用的 UART 通道。 */
    BSP_UART_IMU, /* 惯性测量单元通信使用的 UART 通道。 */
    BSP_UART_DT35, /* DT35 传感器通信使用的 UART 通道。 */
    BSP_UART_UP_STP23L, /* 上方 STP23L 传感器使用的 UART 通道。 */
    BSP_UART_DOWN_STP23L, /* 下方 STP23L 传感器使用的 UART 通道。 */
    BSP_UART_SCRAM, /* 电子急停通信使用的 UART 通道。 */
    BSP_UART_REMOTE, /* 遥控器通信使用的 UART 通道。 */
    BSP_UART_COUNT /* BSP 管理的 UART 通道总数。 */
} bsp_uart_channel_t;

/* 功能：初始化BspUart_Init对应的外设、参数和运行状态。 */
void BspUart_Init(void);

/* 功能：为指定 UART 通道注册接收回调函数。 */
bool BspUart_RegisterRxCallback(bsp_uart_channel_t channel, bsp_uart_rx_callback_t callback);

/* 功能：轮询 UART 接收数据、处理回调并维护发送队列。 */
void BspUart_Task(void *argument);

/* 功能：将数据复制到指定 UART 通道的非阻塞发送队列。 */
bool BspUart_Send(bsp_uart_channel_t channel /* 待发送数据的 UART 通道。 */,
                  const uint8_t *data /* 待发送的字节流。 */,
                  uint16_t size /* 待发送的字节数。 */);
#endif
