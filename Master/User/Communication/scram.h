#ifndef SCRAM_H
#define SCRAM_H

#include <stdint.h>

#define SCRAM_RX_BUFFER_SIZE 1024U
#define SCRAM_LINK_FRAME_HEAD_0 0xA5U
#define SCRAM_LINK_FRAME_HEAD_1 0x5AU
#define SCRAM_LINK_PERIOD_MS 20U
#define SCRAM_PA1_BIT (1U << 3)
#define SCRAM_PA0_BIT (1U << 4)
#define SCRAM_PC3_BIT (1U << 5)

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
void Scram_Task(void);

#endif
