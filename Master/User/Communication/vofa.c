/*
 * @file vofa.c
 * @brief vofa模块的用户接口与实现。
 */

#include "vofa.h"
#include <string.h>

vofa_t vofa = {
    .fdata = {0},
    .tail = {0x00, 0x00, 0x80, 0x7F},
    .tx_state = HAL_OK,
    .send = vofa_send
};

uint8_t tx_buf[CH_COUNT * 4 + 4U];

/* 功能：发送vofa_send对应的数据或控制指令。 */
void vofa_send(vofa_t *vofa, UART_HandleTypeDef *huart)
{
    memcpy(tx_buf,vofa->fdata,sizeof(vofa->fdata));
    memcpy(&tx_buf[sizeof(vofa->fdata)],vofa->tail,4);
    vofa->tx_state = HAL_UART_Transmit_DMA(huart,tx_buf,sizeof(tx_buf));
}