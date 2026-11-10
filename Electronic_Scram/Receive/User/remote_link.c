#include "remote_link.h"
#include "usart.h"

#define REMOTE_RX_DMA_SIZE 128U
static uint8_t rx_dma[REMOTE_RX_DMA_SIZE];
static uint16_t rx_pos;
static uint8_t rx_index;
static volatile uint8_t received_bits;
static volatile uint8_t received_valid;

static void RemoteLink_OutputEmergency(uint8_t bits)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, ((bits & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, ((bits & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, ((bits & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, ((bits & 0x08U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

static void RemoteLink_ParseByte(uint8_t byte)
{
    if (rx_index == 0U) rx_index = (byte == 0xA5U) ? 1U : 0U;
    else if (rx_index == 1U) {
        if (byte == 0x5AU) rx_index = 2U;
        else rx_index = (byte == 0xA5U) ? 1U : 0U;
    } else {
        received_bits = (uint8_t)(byte & 0x0FU);
        received_valid = 1U;
        rx_index = 0U;
    }
}

void RemoteLink_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    RemoteLink_OutputEmergency(0x0FU);
    gpio.Pin = GPIO_PIN_15; gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* E34-2G4H27D: M0=0, M1=0, normal transparent transmission. */
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    rx_pos = rx_index = 0U; received_valid = 0U; received_bits = 0x0FU;
    RemoteLink_OutputEmergency(0x0FU);
    HAL_Delay(10U);
    (void)HAL_UART_Receive_DMA(&huart3, rx_dma, REMOTE_RX_DMA_SIZE);
    if (huart3.hdmarx != NULL) __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

void RemoteLink_Process(void)
{
    uint8_t bits;
    uint16_t write = (uint16_t)(REMOTE_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
    while (rx_pos != write) {
        RemoteLink_ParseByte(rx_dma[rx_pos]);
        rx_pos = (uint16_t)((rx_pos + 1U) % REMOTE_RX_DMA_SIZE);
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); bits = (received_valid == 0U) ? 0x0FU : received_bits;
    RemoteLink_OutputEmergency(bits); __set_PRIMASK(primask);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        (void)HAL_UART_AbortReceive(huart);
        rx_pos = 0U;
        (void)HAL_UART_Receive_DMA(&huart3, rx_dma, REMOTE_RX_DMA_SIZE);
    }
}
