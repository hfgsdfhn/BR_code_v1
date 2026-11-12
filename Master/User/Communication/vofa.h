/*
 * @file vofa.h
 * @brief vofa模块的用户接口与实现。
 */

#ifndef VOFA_H
#define VOFA_H

#include "main.h"
#define CH_COUNT 1   //通道数量

typedef struct vofa vofa_t; /* 成员变量。 */
struct vofa
{
    float fdata[CH_COUNT]; /* 数据缓冲区或数据帧。 */
    uint8_t tail[4]; /* 成员变量。 */
    HAL_StatusTypeDef tx_state; /* 当前工作模式或状态。 */
    void (*send)(vofa_t *vofa, UART_HandleTypeDef *huart);
};

/* 功能：执行 vofa_send 函数对应的模块操作。 */
void vofa_send(vofa_t *vofa, UART_HandleTypeDef *huart);
extern vofa_t vofa; /* 成员变量。 */

#endif
