




#ifndef SCRAM_H
#define SCRAM_H

#include <stdint.h>

#define SCRAM_RX_BUFFER_SIZE 1024U

typedef struct
{
    uint32_t received_bytes;
    uint32_t dropped_bytes;
    uint32_t last_rx_tick;
    uint16_t pending_bytes;
} scram_stats_t;


void Scram_Init(void);

void Scram_RxBytes(const uint8_t *data, uint16_t size);

uint16_t Scram_Read(uint8_t *data, uint16_t capacity);

void Scram_GetStats(scram_stats_t *stats);

#endif
