/*
 * @file remote_link.h
 * @brief remote_link妯″潡鐨勭敤鎴锋帴鍙ｄ笌瀹炵幇銆? */

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
    uint32_t lora_parsed;
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

/* 鍔熻兘锛氬垵濮嬪寲RemoteLink_Init瀵瑰簲鐨勫璁俱€佸弬鏁板拰杩愯鐘舵€併€?*/
void RemoteLink_Init(void);

/* 鍔熻兘锛氳疆璇㈠苟鏇存柊RemoteLink_Process瀵瑰簲鐨勮緭鍏ュ拰鐘舵€併€?*/
void RemoteLink_Process(void);

#ifdef __cplusplus
}
#endif
