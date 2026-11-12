/*
 * @file remote.h
 * @brief remote模块的用户接口与实现。
 */

#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define REMOTE_FRAME_HEADER_0 0xA5U
#define REMOTE_FRAME_HEADER_1 0x5AU
#define REMOTE_FRAME_SIZE 8U

typedef struct
{
    uint8_t joystick; /* 成员变量。 */
    uint8_t group1; /* 成员变量。 */
    uint8_t shoulder; /* 成员变量。 */
    uint8_t group2; /* 成员变量。 */
    uint8_t group3; /* 成员变量。 */
    uint8_t group4; /* 成员变量。 */
    uint32_t received_tick; /* 成员变量。 */
    uint32_t valid_frames; /* 数据缓冲区或数据帧。 */
    uint8_t valid; /* 状态标志。 */
} remote_state_t;


/* 功能：初始化模块并准备外设资源。 */
void Remote_Init(void);

/* 功能：执行 Remote_RxBytes 模块操作并维护相关状态。 */
void Remote_RxBytes(const uint8_t *data, uint16_t size);

/* 功能：记录并处理错误状态。 */
void Remote_UartError(void);

/* 功能：执行 Remote_GetState 模块操作并维护相关状态。 */
uint8_t Remote_GetState(remote_state_t *state);

/* 功能：执行 Remote_IsOnline 模块操作并维护相关状态。 */
uint8_t Remote_IsOnline(uint32_t timeout_ms);

/* 功能：执行 Remote_GetValidFrameCount 模块操作并维护相关状态。 */
uint32_t Remote_GetValidFrameCount(void);

#endif
