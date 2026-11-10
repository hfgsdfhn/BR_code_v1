/*
 * @file bsp_uart.h
 * @brief 应用 UART 通道的公共收发接口。
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
#define UART_RX_BUFFER_SIZE 256U /* 每路 UART 的 DMA 循环接收缓冲区大小，单位：字节。 */
#define CACHE_LINE_SIZE 32U      /* STM32H7 数据缓存地址对齐所使用的缓存行大小，单位：字节。 */
typedef void (*bsp_uart_rx_callback_t)(const uint8_t *data, uint16_t size);
typedef struct
{
    uint8_t buffer[UART_TX_QUEUE_DEPTH][UART_TX_MAX_SIZE]; /* BSP 内部保存的待发送帧。 */
    uint16_t size[UART_TX_QUEUE_DEPTH];                    /* 每个待发送帧的实际字节数。 */
    volatile uint8_t read;                                 /* 当前发送队列读位置。 */
    volatile uint8_t write;                                /* 新帧写入位置。 */
    volatile uint8_t count;                                /* 队列中的帧数量。 */
    volatile bool active;                                  /* 是否已有一帧交给 HAL 异步发送。 */
} bsp_uart_tx_t;                                           /* 单路 UART 的非阻塞发送队列状态。 */
typedef struct
{
    UART_HandleTypeDef *handle;      /* 当前通道对应的 HAL UART 外设句柄。 */
    uint8_t *buffer;                 /* 当前通道的 DMA 循环接收缓冲区。 */
    uint16_t position;               /* 已处理到的接收缓冲区位置。 */
    bsp_uart_rx_callback_t callback; /* 收到新数据时调用的处理函数。 */
    bsp_uart_tx_t tx;                /* 当前通道的异步发送队列。 */
    volatile bool restart_requested; /* 接收出错后请求 UART 任务重启通道。 */
} bsp_uart_t;                        /* UART 外设、DMA 缓冲区和接收状态。 */
typedef enum
{
    BSP_UART_IMU = 0,     /* IMU 小板数据使用 H723 UART8。 */
    BSP_UART_COMPUTER,    /* 上层与小电脑通信使用 UART7。 */
    BSP_UART_SMALL_BOARD, /* Remote wireless data is received by H723 UART9. */
    BSP_UART_DT35,        /* DT35 小板数据使用 H723 UART4 (PA0/PA1)。 */
    BSP_UART_COUNT        /* BSP 管理的 UART 通道总数。 */
} bsp_uart_channel_t;     /* 上层软件可使用的 UART 通信通道。 */
/* 为已注册回调的通道启动 DMA 接收。 */
void BspUart_Init(void);
/* 在 UART 任务启动前注册数据接收处理函数。 */
bool BspUart_RegisterRxCallback(bsp_uart_channel_t channel, bsp_uart_rx_callback_t callback);
/* UART 接收与错误恢复任务。 */
void BspUart_Task(void *argument);
/* 将数据复制到指定 UART 通道的非阻塞发送队列。 */
bool BspUart_Send(bsp_uart_channel_t channel /* 待发送数据的 UART 通道。 */, const uint8_t *data /* 待发送的字节流。 */,
                  uint16_t size /* 待发送的字节数。 */);
#endif

