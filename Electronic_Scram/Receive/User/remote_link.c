#include "remote_link.h"
#include "usart.h"
#include <string.h>

static remote_port_t ports[REMOTE_PORT_COUNT];

static remote_port_t *RemoteLink_FindPort(UART_HandleTypeDef *uart)
{
    uint32_t index;
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (ports[index].uart == uart) return &ports[index];
    }
    return NULL;
}

static HAL_StatusTypeDef RemoteLink_StartReceive(remote_port_t *port)
{
    if (port->uart->hdmarx == NULL || port->uart->hdmatx == NULL) return HAL_ERROR;

    if (port->uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        port->uart->hdmatx->Init.Mode != DMA_NORMAL) return HAL_ERROR;
    return HAL_UARTEx_ReceiveToIdle_DMA(port->uart, port->rx_buffer, REMOTE_RX_SIZE);
}

static uint32_t RemoteLink_UpdateReceive(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t produced;
    uint16_t position;
    uint16_t count;
    __disable_irq();
    position = (uint16_t)((REMOTE_RX_SIZE - __HAL_DMA_GET_COUNTER(port->uart->hdmarx)) % REMOTE_RX_SIZE);
    count = (uint16_t)((position + REMOTE_RX_SIZE - port->rx_position) % REMOTE_RX_SIZE);
    port->rx_position = position;
    port->rx_produced += count;
    produced = port->rx_produced;
    __DMB();
    __set_PRIMASK(primask);
    return produced;
}

static void RemoteLink_Recover(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_UART_Abort(port->uart) != HAL_OK) {
        __set_PRIMASK(primask);
        return;
    }
    port->dropped_bytes += port->tx_write - port->tx_read;
    port->dropped_bytes += port->rx_produced - port->rx_consumed;
    port->rx_produced = port->rx_consumed = 0U;
    port->rx_position = 0U;
    port->tx_write = port->tx_read = 0U;
    port->tx_active_size = 0U;
    port->tx_complete = 0U;
    port->restart_requested = 0U;
    if (RemoteLink_StartReceive(port) != HAL_OK) port->restart_requested = 1U;
    __set_PRIMASK(primask);
}

static void RemoteLink_TransmitLocked(remote_port_t *port)
{
    uint32_t available;
    uint16_t offset;
    uint16_t length;
    HAL_StatusTypeDef status;
    if (port->restart_requested != 0U) return;
    if (port->tx_complete != 0U) {
        port->tx_read += port->tx_active_size;
        port->tx_active_size = 0U;
        port->tx_complete = 0U;
    }
    if (port->tx_active_size != 0U) return;
    available = port->tx_write - port->tx_read;
    if (available == 0U) return;
    offset = (uint16_t)(port->tx_read % REMOTE_TX_SIZE);
    length = (uint16_t)(REMOTE_TX_SIZE - offset);
    if (available < length) length = (uint16_t)available;
    port->tx_active_size = length;

    status = HAL_UART_Transmit_DMA(port->uart, &port->tx_buffer[offset], length);
    if (status != HAL_OK) {
        port->tx_active_size = 0U;
        if (status != HAL_BUSY) port->restart_requested = 1U;
    }
}

static void RemoteLink_Transmit(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RemoteLink_TransmitLocked(port);
    __set_PRIMASK(primask);
}

static void RemoteLink_Forward(remote_port_t *source, remote_port_t *target)
{
    uint32_t produced;
    uint32_t available;
    uint32_t space;
    uint32_t count;
    uint32_t index;
    if (source->restart_requested != 0U || target->restart_requested != 0U) return;
    produced = RemoteLink_UpdateReceive(source);
    __DMB();
    available = produced - source->rx_consumed;
    if (available > REMOTE_RX_SIZE) {
        source->rx_overruns++;
        source->dropped_bytes += available;
        source->rx_consumed = produced;
        return;
    }
    space = REMOTE_TX_SIZE - (target->tx_write - target->tx_read);
    count = (available < space) ? available : space;
    for (index = 0U; index < count; index++) {
        target->tx_buffer[(target->tx_write + index) % REMOTE_TX_SIZE] =
            source->rx_buffer[(source->rx_consumed + index) % REMOTE_RX_SIZE];
    }
    __DMB();
    produced = RemoteLink_UpdateReceive(source);
    if (produced - source->rx_consumed > REMOTE_RX_SIZE) {
        source->rx_overruns++;
        source->dropped_bytes += produced - source->rx_consumed;
        source->rx_consumed = produced;
        return;
    }
    if (source->restart_requested != 0U || target->restart_requested != 0U) return;
    target->tx_write += count;
    source->rx_consumed += count;
}

void RemoteLink_Init(void)
{
    uint32_t index;
    memset(ports, 0, sizeof(ports));
    ports[0].uart = &huart1;
    ports[1].uart = &huart3;
    HAL_Delay(10U);
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (RemoteLink_StartReceive(&ports[index]) != HAL_OK) {
            ports[index].restart_requested = 1U;
        }
    }
}

void RemoteLink_Process(void)
{
    uint32_t index;
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (ports[index].restart_requested != 0U) RemoteLink_Recover(&ports[index]);
        RemoteLink_Transmit(&ports[index]);
    }

    RemoteLink_Forward(&ports[REMOTE_LORA_PORT], &ports[REMOTE_HOST_PORT]);
    RemoteLink_Forward(&ports[REMOTE_HOST_PORT], &ports[REMOTE_LORA_PORT]);
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) RemoteLink_Transmit(&ports[index]);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    (void)size;
    if (port != NULL && port->restart_requested == 0U) {
        (void)RemoteLink_UpdateReceive(port);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    if (port != NULL) {
        port->tx_complete = 1U;
        RemoteLink_Transmit(port);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    if (port != NULL) {
        port->uart_errors++;
        port->restart_requested = 1U;
    }
}
