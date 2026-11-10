/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART10_RX_Pin GPIO_PIN_2
#define USART10_RX_GPIO_Port GPIOE
#define USART10_TX_Pin GPIO_PIN_3
#define USART10_TX_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define BUZZER_EN_Pin GPIO_PIN_15
#define BUZZER_EN_GPIO_Port GPIOC
#define UART4_TX_Pin GPIO_PIN_0
#define UART4_TX_GPIO_Port GPIOA
#define UART4_RX_Pin GPIO_PIN_1
#define UART4_RX_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define UART7_RX_Pin GPIO_PIN_7
#define UART7_RX_GPIO_Port GPIOE
#define UART7_TX_Pin GPIO_PIN_8
#define UART7_TX_GPIO_Port GPIOE
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOB
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOB
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define RS485_1_TX_Pin GPIO_PIN_14
#define RS485_1_TX_GPIO_Port GPIOB
#define RS485_1_RX_Pin GPIO_PIN_15
#define RS485_1_RX_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_10
#define RS485_DE_GPIO_Port GPIOD
#define RS485_RE_Pin GPIO_PIN_11
#define RS485_RE_GPIO_Port GPIOD
#define CAN3_RX_Pin GPIO_PIN_12
#define CAN3_RX_GPIO_Port GPIOD
#define CAN3_TX_Pin GPIO_PIN_13
#define CAN3_TX_GPIO_Port GPIOD
#define UART9_RX_Pin GPIO_PIN_14
#define UART9_RX_GPIO_Port GPIOD
#define UART9_TX_Pin GPIO_PIN_15
#define UART9_TX_GPIO_Port GPIOD
#define USART6_TX_Pin GPIO_PIN_6
#define USART6_TX_GPIO_Port GPIOC
#define USART6_RX_Pin GPIO_PIN_7
#define USART6_RX_GPIO_Port GPIOC
#define CAN1_RX_Pin GPIO_PIN_11
#define CAN1_RX_GPIO_Port GPIOA
#define CAN1_TX_Pin GPIO_PIN_12
#define CAN1_TX_GPIO_Port GPIOA
#define ENCODER_CS_Pin GPIO_PIN_15
#define ENCODER_CS_GPIO_Port GPIOA
#define UART5_TX_Pin GPIO_PIN_12
#define UART5_TX_GPIO_Port GPIOC
#define UART5_RX_Pin GPIO_PIN_2
#define UART5_RX_GPIO_Port GPIOD
#define ENCODER_SPI_SCK_Pin GPIO_PIN_3
#define ENCODER_SPI_SCK_GPIO_Port GPIOB
#define ENCODER_SPI_MISO_Pin GPIO_PIN_4
#define ENCODER_SPI_MISO_GPIO_Port GPIOB
#define ENCODER_SPI_MOSI_Pin GPIO_PIN_5
#define ENCODER_SPI_MOSI_GPIO_Port GPIOB
#define UART8_RX_Pin GPIO_PIN_0
#define UART8_RX_GPIO_Port GPIOE
#define UART8_TX_Pin GPIO_PIN_1
#define UART8_TX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
