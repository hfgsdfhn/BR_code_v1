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
    uint32_t received_bytes; /* 成员变量。 */
    uint32_t dropped_bytes; /* 成员变量。 */
    uint32_t last_rx_tick; /* 成员变量。 */
    uint16_t pending_bytes; /* 成员变量。 */
} scram_stats_t;


/* 功能：初始化模块并准备外设资源。 */
void Scram_Init(void);

/* 功能：执行 Scram_RxBytes 模块操作并维护相关状态。 */
void Scram_RxBytes(const uint8_t *data, uint16_t size);

/* 功能：读取设备参数。 */
uint16_t Scram_Read(uint8_t *data, uint16_t capacity);

/* 功能：执行 Scram_GetStats 模块操作并维护相关状态。 */
void Scram_GetStats(scram_stats_t *stats);

#endif
