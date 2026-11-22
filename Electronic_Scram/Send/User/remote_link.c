#include "remote_link.h"
#include "usart.h"

static uint8_t sw_bits;
static uint8_t sw_last;
static uint8_t sw_prev;
static uint8_t sw_candidate;
static uint8_t sw_sample_count;
static uint8_t sw_initialized;
static uint32_t last_tx_tick;
static uint32_t buzzer_until;
static uint8_t link_ready;
static uint8_t tx_frame[3];

void RemoteLink_Init(void)
{
    sw_bits = 0U; sw_last = 0xFFU; sw_prev = 0U; sw_candidate = 0U; sw_sample_count = 0U; sw_initialized = 0U;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    link_ready = 1U;
}

void RemoteLink_PollSwitches(void)
{
    uint8_t sample;

    sample = 0U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) sample |= 1U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) sample |= 2U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) sample |= 4U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) sample |= 8U;

    if (sw_initialized == 0U) {
        sw_prev = sample;
        sw_candidate = sample;
        sw_sample_count = 1U;
        sw_bits = sample;
        sw_initialized = 1U;
        return;
    }

    if (sample != sw_candidate) {
        sw_candidate = sample;
        sw_sample_count = 1U;
    } else if (sw_sample_count < 3U) {
        sw_sample_count++;
    }

    if (sw_sample_count >= 3U && sw_candidate != sw_prev) {
        sw_prev = sw_candidate;
        sw_bits = sw_prev;
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        buzzer_until = HAL_GetTick() + 100U;
    } else {
        sw_bits = sw_prev;
    }
}

void RemoteLink_Process(void)
{
    uint32_t now;
    if (link_ready == 0U) return;
    now = HAL_GetTick();
    if (buzzer_until != 0U && (int32_t)(now - buzzer_until) >= 0) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        buzzer_until = 0U;
    }
    if ((sw_bits == sw_last) && ((uint32_t)(now - last_tx_tick) < 100U)) return;
    if (huart1.gState != HAL_UART_STATE_READY) return;
    tx_frame[0] = 0xA5U; tx_frame[1] = 0x5AU; tx_frame[2] = (uint8_t)(sw_bits & 0x0FU);
    if (HAL_UART_Transmit_DMA(&huart1, tx_frame, sizeof(tx_frame)) == HAL_OK) {
        sw_last = sw_bits; last_tx_tick = now;
    }
}

void RemoteLink_ForwardRawFrame(void) {}
