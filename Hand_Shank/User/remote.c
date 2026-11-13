/*
 * @file remote.c
 * @brief remote妯″潡鐨勭敤鎴锋帴鍙ｄ笌瀹炵幇銆? */

#include "remote.h"
/* E34-2G4H27D：M0=0、M1=0 为透明传输；模块参数由上位机配置，固件不发送配置命令。 */
#include <string.h>


extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;


/* 鍔熻兘锛氬鐞嗗璁惧洖璋冧簨浠躲€?*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}


static uint8_t remote_tx_ring[REMOTE_TX_QUEUE_DEPTH][REMOTE_FRAME_SIZE];
static volatile uint8_t remote_tx_read;
static uint8_t remote_tx_write;
static volatile uint8_t remote_tx_count;
static volatile uint8_t remote_tx_active;
static volatile uint8_t remote_tx_error;
static uint32_t remote_next_tick;

#define REMOTE_KEY_COUNT 27U

typedef struct
{
    uint8_t candidate;
    uint8_t stable;
    uint8_t count;
    uint8_t initialized;
} remote_key_filter_t;

static remote_key_filter_t remote_key_filter[REMOTE_KEY_COUNT];
static uint16_t remote_adc_filtered[4];
static uint8_t remote_adc_initialized[4];


/* 鍔熻兘锛氳鍙栬澶囧弬鏁般€?*/
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


/* 鍔熻兘锛氭墽琛?Remote_AdcToNibble 妯″潡鎿嶄綔骞剁淮鎶ょ浉鍏崇姸鎬併€?*/
static uint8_t Remote_AdcToNibble(uint16_t value)
{
    return (uint8_t)((((uint32_t)value * 15U) + 2047U) / 4095U);
}

static uint8_t Remote_ReadAdcFiltered(uint32_t channel, uint8_t index)
{
    const uint16_t sample = Remote_ReadAdc(channel);
    if (remote_adc_initialized[index] == 0U)
    {
        remote_adc_filtered[index] = sample;
        remote_adc_initialized[index] = 1U;
    }
    else
    {
        remote_adc_filtered[index] = (uint16_t)(((uint32_t)remote_adc_filtered[index] * 3U + sample + 2U) / 4U);
    }
    return Remote_AdcToNibble(remote_adc_filtered[index]);
}

static uint8_t Remote_Key(GPIO_TypeDef *port, uint16_t pin, uint8_t index)
{
    const uint8_t sample = (uint8_t)(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
    remote_key_filter_t *filter = &remote_key_filter[index];

    if (filter->initialized == 0U)
    {
        filter->candidate = sample;
        filter->stable = sample;
        filter->count = REMOTE_KEY_DEBOUNCE_SAMPLES;
        filter->initialized = 1U;
    }
    else if (sample != filter->candidate)
    {
        filter->candidate = sample;
        filter->count = 1U;
    }
    else if (filter->count < REMOTE_KEY_DEBOUNCE_SAMPLES)
    {
        filter->count++;
    }

    if (filter->count >= REMOTE_KEY_DEBOUNCE_SAMPLES)
        filter->stable = filter->candidate;

    return filter->stable;
}

static void Remote_BuildFrame(uint8_t *remote_tx_frame)
{
    static const uint16_t pd_group2[] = {
        GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12,
        GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
    };
    uint8_t i;

    remote_tx_frame[0] = REMOTE_FRAME_HEADER_0;
    remote_tx_frame[1] = REMOTE_FRAME_HEADER_1;

    remote_tx_frame[2] = (uint8_t)(Remote_ReadAdcFiltered(ADC_CHANNEL_14, 0U) |
                                   (uint8_t)(Remote_ReadAdcFiltered(ADC_CHANNEL_15, 1U) << 4U));

    remote_tx_frame[3] = 0U;
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_6, 0U) << 0U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_5, 1U) << 1U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_4, 2U) << 2U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_1, 3U) << 3U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_0, 4U) << 4U);
    remote_tx_frame[3] |= (uint8_t)(Remote_Key(GPIOC, GPIO_PIN_3, 5U) << 5U);


    remote_tx_frame[4] = (uint8_t)(Remote_ReadAdcFiltered(ADC_CHANNEL_7, 2U) |
                                   (uint8_t)(Remote_ReadAdcFiltered(ADC_CHANNEL_11, 3U) << 4U));

    remote_tx_frame[5] = 0U;
    for (i = 0U; i < 6U; ++i)
        remote_tx_frame[5] |= (uint8_t)(Remote_Key(GPIOD, pd_group2[i], (uint8_t)(6U + i)) << i);
    remote_tx_frame[5] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_8, 12U) << 6U);
    remote_tx_frame[5] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_11, 13U) << 7U);

    remote_tx_frame[6] = 0U;
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_12, 14U) << 0U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_6, 15U) << 1U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_3, 16U) << 2U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_4, 17U) << 3U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_5, 18U) << 4U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_2, 19U) << 5U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_1, 20U) << 6U);
    remote_tx_frame[6] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_0, 21U) << 7U);

    remote_tx_frame[7] = 0U;
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_3, 22U) << 0U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_4, 23U) << 1U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_7, 24U) << 2U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_6, 25U) << 3U);
    remote_tx_frame[7] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_5, 26U) << 4U);
}


/* 鍔熻兘锛氬惎鍔ㄥ紓姝ヤ紶杈撴垨璁惧鍔ㄤ綔銆?*/
static void Remote_StartTx(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (remote_tx_count != 0U && remote_tx_active == 0U && remote_tx_error == 0U)
    {
        remote_tx_active = 1U;
        if (HAL_UART_Transmit_DMA(&huart6, remote_tx_ring[remote_tx_read], REMOTE_FRAME_SIZE) != HAL_OK)
            remote_tx_active = 0U;
    }
    __set_PRIMASK(primask);
}


/* 鍔熻兘锛氬鐞嗗璁惧洖璋冧簨浠躲€?*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart6 || remote_tx_active == 0U) return;
    remote_tx_read = (uint8_t)((remote_tx_read + 1U) % REMOTE_TX_QUEUE_DEPTH);
    remote_tx_count--;
    remote_tx_active = 0U;
}


/* 鍔熻兘锛氬鐞嗗璁惧洖璋冧簨浠躲€?*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6 && (huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
        remote_tx_error = 1U;
}


/* 鍔熻兘锛氬垵濮嬪寲妯″潡骞跺噯澶囧璁捐祫婧愩€?*/
void Remote_Init(void)
{
    remote_tx_read = 0U;
    remote_tx_write = 0U;
    remote_tx_count = 0U;
    remote_tx_active = 0U;
    remote_tx_error = 0U;
    remote_next_tick = HAL_GetTick();
    memset(remote_key_filter, 0, sizeof(remote_key_filter));
    memset(remote_adc_filtered, 0, sizeof(remote_adc_filtered));
    memset(remote_adc_initialized, 0, sizeof(remote_adc_initialized));
}


/* 鍔熻兘锛氭墽琛?Remote_Loop 妯″潡鎿嶄綔骞剁淮鎶ょ浉鍏崇姸鎬併€?*/
void Remote_Loop(void)
{
    uint8_t frame[REMOTE_FRAME_SIZE];
    uint32_t primask;
    const uint32_t now = HAL_GetTick();
    if (remote_tx_error != 0U)
    {

        (void)HAL_UART_AbortTransmit(&huart6);
        primask = __get_PRIMASK();
        __disable_irq();
        remote_tx_read = remote_tx_write;
        remote_tx_count = 0U;
        remote_tx_active = 0U;
        remote_tx_error = 0U;
        __set_PRIMASK(primask);
    }
    Remote_StartTx();
    if ((int32_t)(now - remote_next_tick) < 0) return;
    remote_next_tick = now + REMOTE_TX_PERIOD_MS;
    if (remote_tx_count >= REMOTE_TX_QUEUE_DEPTH) return;
    Remote_BuildFrame(frame);
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(remote_tx_ring[remote_tx_write], frame, REMOTE_FRAME_SIZE);
    remote_tx_write = (uint8_t)((remote_tx_write + 1U) % REMOTE_TX_QUEUE_DEPTH);
    remote_tx_count++;
    __set_PRIMASK(primask);
    Remote_StartTx();
}


