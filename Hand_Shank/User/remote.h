/*
 * @file remote.h
 * @brief remote模块的用户接口与实现。
 */

#ifndef HAND_REMOTE_H
#define HAND_REMOTE_H

#include <stdint.h>
#include "main.h"

#define REMOTE_FRAME_HEADER_0 0xA5U
#define REMOTE_FRAME_HEADER_1 0x5AU
#define REMOTE_FRAME_SIZE 8U
#define REMOTE_TX_PERIOD_MS 20U
#define REMOTE_TX_QUEUE_DEPTH 4U

/* 功能：执行 Remote_Init 函数对应的模块操作。 */
void Remote_Init(void);

/* 功能：执行 Remote_Loop 函数对应的模块操作。 */
void Remote_Loop(void);

#endif
