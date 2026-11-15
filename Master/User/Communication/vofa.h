/*
 * @file vofa.h
 * @brief vofa模块的用户接口与实现。
 */

#ifndef VOFA_H
#define VOFA_H

#include "main.h"
#define CH_COUNT 1   //通道数量

typedef struct vofa vofa_t;
struct vofa
{
    float fdata[CH_COUNT];
    uint8_t tail[4];
    HAL_StatusTypeDef tx_state; /* 当前工作模式或状态。 */
    void (*send)(vofa_t *vofa, UART_HandleTypeDef *huart);
};

/* 功能：发送vofa_send对应的数据或控制指令。 */
void vofa_send(vofa_t *vofa, UART_HandleTypeDef *huart);
extern vofa_t vofa;

#endif
