#include "usart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static void UART_CommonInit(UART_HandleTypeDef *h, USART_TypeDef *instance)
{
  h->Instance = instance;
  h->Init.BaudRate = 115200;
  h->Init.WordLength = UART_WORDLENGTH_8B;
  h->Init.StopBits = UART_STOPBITS_1;
  h->Init.Parity = UART_PARITY_NONE;
  h->Init.Mode = UART_MODE_TX_RX;
  h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  h->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(h) != HAL_OK)
    Error_Handler();
}

void MX_USART1_UART_Init(void)
{
  UART_CommonInit(&huart1, USART1);
}
void MX_USART2_UART_Init(void)
{
  UART_CommonInit(&huart2, USART2);
}
void MX_USART3_UART_Init(void)
{
  UART_CommonInit(&huart3, USART3);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = U4_7_PA9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = U4_8_PA10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = UART2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART2_TX_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = UART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART2_RX_GPIO_Port, &GPIO_InitStruct);
  }
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = LORA_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_TX_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LORA_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_RX_GPIO_Port, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
  if (uartHandle->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, U4_7_PA9_Pin | U4_8_PA10_Pin);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, UART2_TX_Pin | UART2_RX_Pin);
  }
  else if (uartHandle->Instance == USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, LORA_TX_Pin | LORA_RX_Pin);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}
