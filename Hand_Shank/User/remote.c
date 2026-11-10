#include "remote.h"

/* E22-400T22D: externally configured for 115200 8N1 transparent UART.
 * M0/M1 must be held low by hardware; this board has no mode GPIO mapping.
 */
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;

/* HAL ADC expects this legacy callback when injected callbacks are enabled. */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

static uint8_t remote_tx_frame[REMOTE_FRAME_SIZE];
static uint32_t remote_next_tick;

static uint16_t Remote_ReadAdc(uint32_t channel)
{
    ADC_ChannelConfTypeDef config = {0};
    uint16_t value = 0U;
    config.Channel = channel;
    config.Rank = 1U;
    config.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK)
        return 0U;
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return 0U;
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK)
        value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return value;
}

static uint8_t Remote_AdcToNibble(uint16_t value)
{
    return (uint8_t)((((uint32_t)value * 15U) + 2047U) / 4095U);
}

static uint8_t Remote_Key(GPIO_TypeDef *port, uint16_t pin)
{
    return (uint8_t)(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

static void Remote_BuildFrame(void)
{
    static const uint16_t pd_group2[] = {
        GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12,
        GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
    };
    uint8_t i;

    remote_tx_frame[0] = REMOTE_FRAME_HEADER_0;
    remote_tx_frame[1] = REMOTE_FRAME_HEADER_1;
    remote_tx_frame[2] = (uint8_t)(Remote_AdcToNibble(Remote_ReadAdc(ADC_CHANNEL_14)) |
                                   (uint8_t)(Remote_AdcToNibble(Remote_ReadAdc(ADC_CHANNEL_15)) << 4U));

remote_tx_frame[3] = 0U;
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_6) << 0U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_5) << 1U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_4) << 2U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_1) << 3U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_0) << 4U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOC, GPIO_PIN_3) << 5U);

    remote_tx_frame[4] = (uint8_t)(Remote_AdcToNibble(Remote_ReadAdc(ADC_CHANNEL_7)) |
                                   (uint8_t)(Remote_AdcToNibble(Remote_ReadAdc(ADC_CHANNEL_11)) << 4U));

    remote_tx_frame[5] = 0U;
    for (i = 0U; i < 6U; ++i)
        remote_tx_frame[5] |= (uint8_t)(Remote_Key(GPIOD, pd_group2[i]) << i);
    remote_tx_frame[5] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_8) << 6U);
    remote_tx_frame[5] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_11) << 7U);

    remote_tx_frame[6] = 0U;
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_12) << 0U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_6) << 1U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_3) << 2U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_4) << 3U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_5) << 4U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_2) << 5U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_1) << 6U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_0) << 7U);

    remote_tx_frame[7] = 0U;
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_3) << 0U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_4) << 1U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_7) << 2U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_6) << 3U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_5) << 4U);
}

void Remote_Init(void)
{
    remote_next_tick = HAL_GetTick();
    Remote_BuildFrame();
}

void Remote_Loop(void)
{
    const uint32_t now = HAL_GetTick();
    if ((int32_t)(now - remote_next_tick) < 0)
        return;
    remote_next_tick = now + REMOTE_TX_PERIOD_MS;
    if (huart6.gState != HAL_UART_STATE_READY)
        return;
    Remote_BuildFrame();
    (void)HAL_UART_Transmit_DMA(&huart6, remote_tx_frame, REMOTE_FRAME_SIZE);
}

