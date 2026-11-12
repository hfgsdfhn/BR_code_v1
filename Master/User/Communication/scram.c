




#include "scram.h"
#include "bsp_uart.h"
#include <string.h>

static uint8_t rx_queue[SCRAM_RX_BUFFER_SIZE];
static uint32_t rx_write;
static uint32_t rx_read;
static scram_stats_t rx_stats;


void Scram_Init(void)
{
    rx_write = 0U;
    rx_read = 0U;
    memset(&rx_stats, 0, sizeof(rx_stats));
    (void)BspUart_RegisterRxCallback(BSP_UART_SCRAM, Scram_RxBytes);
}


void Scram_RxBytes(const uint8_t *data, uint16_t size)
{
    uint32_t primask;
    uint32_t space;
    uint16_t count, offset, first;
    if (data == NULL || size == 0U) return;
    primask = __get_PRIMASK();
    __disable_irq();
    space = SCRAM_RX_BUFFER_SIZE - (rx_write - rx_read);
    count = (size < space) ? size : (uint16_t)space;
    offset = (uint16_t)(rx_write % SCRAM_RX_BUFFER_SIZE);
    first = (uint16_t)(SCRAM_RX_BUFFER_SIZE - offset);
    if (first > count) first = count;
    memcpy(&rx_queue[offset], data, first);
    memcpy(rx_queue, &data[first], count - first);
    rx_write += count;
    rx_stats.received_bytes += size;
    rx_stats.dropped_bytes += size - count;
    rx_stats.last_rx_tick = HAL_GetTick();
    __set_PRIMASK(primask);
}


uint16_t Scram_Read(uint8_t *data, uint16_t capacity)
{
    uint32_t primask, available;
    uint16_t count, offset, first;
    if (data == NULL || capacity == 0U) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    available = rx_write - rx_read;
    count = (capacity < available) ? capacity : (uint16_t)available;
    offset = (uint16_t)(rx_read % SCRAM_RX_BUFFER_SIZE);
    first = (uint16_t)(SCRAM_RX_BUFFER_SIZE - offset);
    if (first > count) first = count;
    memcpy(data, &rx_queue[offset], first);
    memcpy(&data[first], rx_queue, count - first);
    rx_read += count;
    __set_PRIMASK(primask);
    return count;
}


void Scram_GetStats(scram_stats_t *stats)
{
    uint32_t primask;
    if (stats == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *stats = rx_stats;
    stats->pending_bytes = (uint16_t)(rx_write - rx_read);
    __set_PRIMASK(primask);
}
