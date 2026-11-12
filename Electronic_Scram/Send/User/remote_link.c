/*
 * @file remote_link.c
 * @brief remote_link模块的用户接口与实现。
 */

#include "remote_link.h"
#include "usart.h"

static uint8_t sw_bits;
static uint8_t sw_last;
static uint32_t last_tx_tick;
static uint8_t link_ready;
static uint8_t tx_frame[3];

/* 功能：执行 RemoteLink_Init 函数对应的模块操作。 */
void RemoteLink_Init(void)
{

    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    sw_bits = 0U; sw_last = 0xFFU; last_tx_tick = 0U; link_ready = 1U;
}

/* 功能：执行 RemoteLink_PollSwitches 函数对应的模块操作。 */
void RemoteLink_PollSwitches(void)
{
    sw_bits = 0U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) sw_bits |= 1U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) sw_bits |= 2U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) sw_bits |= 4U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) sw_bits |= 8U;
}

/* 功能：执行 RemoteLink_Process 函数对应的模块操作。 */
void RemoteLink_Process(void)
{
    uint32_t now;
    if (link_ready == 0U) return;
    now = HAL_GetTick();
    if ((sw_bits == sw_last) && ((uint32_t)(now - last_tx_tick) < 100U)) return;
    if (huart1.gState != HAL_UART_STATE_READY) return;
    tx_frame[0] = 0xA5U; tx_frame[1] = 0x5AU; tx_frame[2] = (uint8_t)(sw_bits & 0x0FU);
    if (HAL_UART_Transmit_DMA(&huart1, tx_frame, sizeof(tx_frame)) == HAL_OK) {
        sw_last = sw_bits; last_tx_tick = now;
    }
}

/* 功能：执行 RemoteLink_ForwardRawFrame 函数对应的模块操作。 */
void RemoteLink_ForwardRawFrame(void) {}
