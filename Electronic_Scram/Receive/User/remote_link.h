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
    struct __UART_HandleTypeDef *uart;
    uint8_t rx_buffer[REMOTE_RX_SIZE];
    uint8_t tx_buffer[REMOTE_TX_SIZE];
    volatile uint32_t rx_produced;
    uint32_t rx_consumed;
    volatile uint16_t rx_position;
    volatile uint32_t tx_write;
    volatile uint32_t tx_read;
    volatile uint16_t tx_active_size;
    volatile uint8_t tx_complete;
    volatile uint8_t restart_requested;
    volatile uint32_t uart_errors;
    volatile uint32_t rx_overruns;
    volatile uint32_t dropped_bytes;
} remote_port_t;

#ifdef __cplusplus
extern "C"
{
#endif

/* 功能：初始化RemoteLink_Init对应的外设、参数和运行状态。 */
void RemoteLink_Init(void);

/* 功能：轮询并更新RemoteLink_Process对应的输入和状态。 */
void RemoteLink_Process(void);

#ifdef __cplusplus
}
#endif
