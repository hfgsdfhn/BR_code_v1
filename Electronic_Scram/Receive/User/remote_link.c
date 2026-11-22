#include "remote_link.h"
#include "usart.h"
#include <string.h>

#define REMOTE_LINK_TIMEOUT_MS 300U

remote_port_t remote_lora_port;
static uint8_t lora_frame_state;
static uint32_t last_frame_tick;
static uint8_t outputs_forced_high;

static void RemoteLink_SetOutputsHigh(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}

static void RemoteLink_ApplyLoraStatus(uint8_t status)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5,
                      (status & (1U << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,
                      (status & (1U << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,
                      (status & (1U << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void RemoteLink_ProcessLoraByte(uint8_t byte)
{
    if (lora_frame_state == 0U) {
        if (byte == 0xA5U) lora_frame_state = 1U;
    } else if (lora_frame_state == 1U) {
        if (byte == 0x5AU) lora_frame_state = 2U;
        else lora_frame_state = (byte == 0xA5U) ? 1U : 0U;
    } else {
        /* Only the low nibble is meaningful (Send transmits PA0..PA3); a byte
         * with any high bit set means the frame was truncated or corrupted. */
        if ((byte & 0xF0U) == 0U) {
            RemoteLink_ApplyLoraStatus(byte);
            last_frame_tick = HAL_GetTick();
            outputs_forced_high = 0U;
        }
        lora_frame_state = 0U;
    }
}

static HAL_StatusTypeDef RemoteLink_StartReceive(remote_port_t *port)
{
    if (port->uart->hdmarx == NULL) return HAL_ERROR;
    if (port->uart->hdmarx->Init.Mode != DMA_CIRCULAR) return HAL_ERROR;
    return HAL_UART_Receive_DMA(port->uart, port->rx_buffer, REMOTE_RX_SIZE);
}

static uint32_t RemoteLink_UpdateReceive(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t position;
    uint16_t count;
    __disable_irq();
    position = (uint16_t)((REMOTE_RX_SIZE - __HAL_DMA_GET_COUNTER(port->uart->hdmarx)) % REMOTE_RX_SIZE);
    count = (uint16_t)((position + REMOTE_RX_SIZE - port->rx_position) % REMOTE_RX_SIZE);
    port->rx_position = position;
    port->rx_produced += count;
    __DMB();
    __set_PRIMASK(primask);
    return port->rx_produced;
}

static void RemoteLink_Recover(remote_port_t *port)
{
    (void)HAL_UART_AbortReceive(port->uart);
    port->rx_produced = 0U;
    port->rx_consumed = 0U;
    port->rx_position = 0U;
    port->restart_requested = (RemoteLink_StartReceive(port) == HAL_OK) ? 0U : 1U;
}

void RemoteLink_Init(void)
{
    memset(&remote_lora_port, 0, sizeof(remote_lora_port));
    remote_lora_port.uart = &huart3;
    lora_frame_state = 0U;
    last_frame_tick = HAL_GetTick();
    outputs_forced_high = 1U;
    RemoteLink_SetOutputsHigh();
    HAL_Delay(10U);
    if (RemoteLink_StartReceive(&remote_lora_port) != HAL_OK)
        remote_lora_port.restart_requested = 1U;
}

void RemoteLink_Process(void)
{
    uint32_t produced;
    uint32_t index;
    if (remote_lora_port.restart_requested != 0U) RemoteLink_Recover(&remote_lora_port);
    produced = RemoteLink_UpdateReceive(&remote_lora_port);
    while (remote_lora_port.rx_consumed != produced) {
        index = remote_lora_port.rx_consumed % REMOTE_RX_SIZE;
        RemoteLink_ProcessLoraByte(remote_lora_port.rx_buffer[index]);
        remote_lora_port.rx_consumed++;
    }
    if ((uint32_t)(HAL_GetTick() - last_frame_tick) >= REMOTE_LINK_TIMEOUT_MS) {
        if (outputs_forced_high == 0U) {
            RemoteLink_SetOutputsHigh();
            outputs_forced_high = 1U;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)size;
    if (uart == remote_lora_port.uart && remote_lora_port.restart_requested == 0U)
        (void)RemoteLink_UpdateReceive(&remote_lora_port);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == remote_lora_port.uart) {
        remote_lora_port.uart_errors++;
        remote_lora_port.restart_requested = 1U;
    }
}
