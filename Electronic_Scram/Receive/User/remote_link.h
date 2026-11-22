#pragma once
#include <stdint.h>
#define REMOTE_RX_SIZE 512U

typedef struct
{
    struct __UART_HandleTypeDef *uart;
    uint8_t rx_buffer[REMOTE_RX_SIZE];
    volatile uint32_t rx_produced;
    uint32_t rx_consumed;
    volatile uint16_t rx_position;
    volatile uint8_t restart_requested;
    volatile uint32_t uart_errors;
} remote_port_t;

extern remote_port_t remote_lora_port;

#ifdef __cplusplus
extern "C" {
#endif
void RemoteLink_Init(void);
void RemoteLink_Process(void);
#ifdef __cplusplus
}
#endif
