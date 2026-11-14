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
#define UART_RESTART_ALL_FLAGS ((1UL << BSP_UART_COUNT /* UART 通道总数。 */
#define UART_RX_BUFFER_SIZE 256U
#define CACHE_LINE_SIZE 32U
/* 代码说明：相关模块实现。 */
typedef uint8_t (*bsp_uart_rx_callback_t)(const uint8_t *data, uint16_t size);
typedef struct
{
    uint8_t buffer[UART_TX_QUEUE_DEPTH][UART_TX_MAX_SIZE];
    uint16_t size[UART_TX_QUEUE_DEPTH]; /* 代码说明：相关模块实现。 */
    volatile uint8_t read; /* 代码说明：相关模块实现。 */
    volatile uint8_t write; /* 代码说明：相关模块实现。 */
    volatile uint8_t count; /* 代码说明：相关模块实现。 */
    volatile bool active; /* 代码说明：相关模块实现。 */
} bsp_uart_tx_t;
typedef struct
{
    UART_HandleTypeDef *handle; /* 代码说明：相关模块实现。 */
    uint8_t *buffer; /* 代码说明：相关模块实现。 */
    volatile uint32_t rx_produced; /* 代码说明：相关模块实现。 */
    uint32_t rx_consumed; /* 代码说明：相关模块实现。 */
    uint32_t rx_overruns; /* 代码说明：相关模块实现。 */
    uint16_t position; /* 代码说明：相关模块实现。 */
    bsp_uart_rx_callback_t callback; /* 浜嬩欢鍥炶皟鍑芥暟銆?*/
    bsp_uart_tx_t tx; /* 代码说明：相关模块实现。 */
    volatile bool restart_requested; /* 代码说明：相关模块实现。 */
} bsp_uart_t;
typedef enum
{
    BSP_UART_COMPUTER = 0, /* 上位机通信使用的 UART 通道。 */
    BSP_UART_IMU, /* 惯性测量单元通信使用的 UART 通道。 */
    BSP_UART_DT35, /* DT35 传感器通信使用的 UART 通道。 */
    BSP_UART_UP_STP23L, /* 上方 STP23L 传感器通信使用的 UART 通道。 */
    BSP_UART_DOWN_STP23L, /* 下方 STP23L 传感器通信使用的 UART 通道。 */
    BSP_UART_SCRAM, /* 电子急停通信使用的 UART 通道。 */
    BSP_UART_REMOTE, /* 遥控器通信使用的 UART 通道。 */
    BSP_UART_COUNT /* UART 通道总数。 */
} bsp_uart_channel_t;

/* 代码说明：相关模块实现。 */
void BspUart_Init(void);

/* 代码说明：相关模块实现。 */
bool BspUart_RegisterRxCallback(bsp_uart_channel_t channel, bsp_uart_rx_callback_t callback);

/* 代码说明：相关模块实现。 */
void BspUart_Task(void *argument);

/* 代码说明：相关模块实现。 */
bool BspUart_Send(bsp_uart_channel_t channel /* 待发送数据的 UART 通道。 */,
                  const uint8_t *data /* 待发送的字节流。 */,
                  uint16_t size /* 待发送的字节数。 */);
#endif
