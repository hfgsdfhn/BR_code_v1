/*
 * @file scram.h
 * @brief scram模块的用户接口与实现。
 */

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

/* 功能：初始化Scram_Init对应的外设、参数和运行状态。 */
void Scram_Init(void);

/* 功能：接收并解析Scram_RxBytes对应的输入数据，更新模块状态。 */
void Scram_RxBytes(const uint8_t *data, uint16_t size);

/* 功能：处理Scram_Read对应的输入并更新相关状态。 */
uint16_t Scram_Read(uint8_t *data, uint16_t capacity);

/* 功能：读取Scram_GetStats对应的数据或运行状态。 */
void Scram_GetStats(scram_stats_t *stats);

#endif
