/*
 * @file remote_link.h
 * @brief remote_link模块的用户接口与实现。
 */

#pragma once

#include <stdint.h>
#define REMOTE_RX_SIZE 512U
#define REMOTE_TX_SIZE 1024U
#define REMOTE_PORT_COUNT 2U
#define REMOTE_LORA_PORT 1U
#define REMOTE_HOST_PORT 0U

typedef struct
{
    struct __UART_HandleTypeDef *uart; /* 成员变量。 */
    uint8_t rx_buffer[REMOTE_RX_SIZE]; /* 数据缓冲区或数据帧。 */
    uint8_t tx_buffer[REMOTE_TX_SIZE]; /* 数据缓冲区或数据帧。 */
    volatile uint32_t rx_produced; /* 成员变量。 */
    uint32_t rx_consumed; /* 成员变量。 */
    volatile uint16_t rx_position; /* 成员变量。 */
    volatile uint32_t tx_write; /* 成员变量。 */
    volatile uint32_t tx_read; /* 成员变量。 */
    volatile uint16_t tx_active_size; /* 数据长度或数量。 */
    volatile uint8_t tx_complete; /* 成员变量。 */
    volatile uint8_t restart_requested; /* 成员变量。 */
    volatile uint32_t uart_errors; /* 成员变量。 */
    volatile uint32_t rx_overruns; /* 成员变量。 */
    volatile uint32_t dropped_bytes; /* 成员变量。 */
} remote_port_t;

#ifdef __cplusplus
extern "C"
{
#endif

/* 功能：初始化模块并准备外设资源。 */
void RemoteLink_Init(void);

/* 功能：处理当前输入并更新模块状态。 */
void RemoteLink_Process(void);

#ifdef __cplusplus
}
#endif
