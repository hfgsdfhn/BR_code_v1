



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
    uint8_t buffer[UART_TX_QUEUE_DEPTH][UART_TX_MAX_SIZE];
    uint16_t size[UART_TX_QUEUE_DEPTH];
    volatile uint8_t read;
    volatile uint8_t write;
    volatile uint8_t count;
    volatile bool active;
} bsp_uart_tx_t;
typedef struct
{
    UART_HandleTypeDef *handle;
    uint8_t *buffer;
    volatile uint32_t rx_produced;
    uint32_t rx_consumed;
    uint32_t rx_overruns;
    uint16_t position;
    bsp_uart_rx_callback_t callback;
    bsp_uart_tx_t tx;
    volatile bool restart_requested;
} bsp_uart_t;
typedef enum
{
    BSP_UART_COMPUTER = 0,
    BSP_UART_IMU,
    BSP_UART_DT35,
    BSP_UART_UP_STP23L,
    BSP_UART_DOWN_STP23L,
    BSP_UART_SCRAM,
    BSP_UART_REMOTE,
    BSP_UART_COUNT
} bsp_uart_channel_t;

void BspUart_Init(void);

bool BspUart_RegisterRxCallback(bsp_uart_channel_t channel, bsp_uart_rx_callback_t callback);

void BspUart_Task(void *argument);

bool BspUart_Send(bsp_uart_channel_t channel                          , const uint8_t *data                   ,
                  uint16_t size                   );
#endif
