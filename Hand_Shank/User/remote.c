/*
 * @file remote.c
 * @brief remote 模块的用户接口与实现。
 *
 * 帧格式（16 字节，小端）：
 *   [0]  0xAA
 *   [1]  0x55
 *   [2..3]  摇杆1 X  int16  (-1000~+1000)
 *   [4..5]  摇杆1 Y  int16
 *   [6..7]  摇杆2 X  int16
 *   [8..9]  摇杆2 Y  int16
 *   [10..12] 按键位（22 个键）
 *   [13] 预留（可放 CRC8）
 *   [14] 协议版本
 *   [15] 0x5A 帧尾
 */

#include "remote.h"
#include <string.h>

extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;

uint8_t frame[REMOTE_FRAME_SIZE];

/* ---------------- 发送环形队列 ---------------- */
static uint8_t remote_tx_ring[REMOTE_TX_QUEUE_DEPTH][REMOTE_FRAME_SIZE];
static volatile uint8_t remote_tx_read;
static uint8_t remote_tx_write;
static volatile uint8_t remote_tx_count;
static volatile uint8_t remote_tx_active;
static volatile uint8_t remote_tx_error;
static uint32_t remote_next_tick;

/* ============================================================
 *  ADC / GPIO 读取
 * ============================================================ */

/* 执行 HAL_ADCEx_InjectedConvCpltCallback 对应的模块操作 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

/* 读取一次 ADC 通道（单次轮询） */
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

/* 按键读取，按下返回 1 */
static uint8_t Remote_Key(GPIO_TypeDef *port, uint16_t pin)
{
    return (uint8_t)(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

/* ============================================================
 *  摇杆：ADC -> int16 定点 (-1000 ~ +1000)
 * ============================================================ */

/*
 * 将 12 位 ADC 值归一化到 [-REMOTE_STICK_FULL_SCALE, +REMOTE_STICK_FULL_SCALE]。
 * center 为摇杆回中时的 ADC 值，deadzone 为中心死区（ADC 计数）。
 * 正向和负向分别按剩余量程线性映射，保证两端都能打到满量程 ±1000。
 */
static int16_t Remote_ReadStick(uint32_t channel, uint16_t center, uint16_t deadzone)
{
    uint16_t raw = Remote_ReadAdc(channel);
    int32_t delta = (int32_t)raw - (int32_t)center;
    float norm;

    /* 中心死区 */
    if (delta > -(int32_t)deadzone && delta < (int32_t)deadzone)
        return 0;

    if (delta > 0)
        norm = (float)delta / (float)(4095 - center);
    else
        norm = (float)delta / (float)center;

    if (norm > 1.0f)  norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    return (int16_t)(norm * (float)REMOTE_STICK_FULL_SCALE);
}

/* 小端写入 int16 */
static void Remote_PutInt16(uint8_t *dst, int16_t v)
{
    uint16_t u = (uint16_t)v;
    dst[0] = (uint8_t)(u & 0xFF);
    dst[1] = (uint8_t)((u >> 8) & 0xFF);
}

/* ============================================================
 *  帧构建
 * ============================================================ */

static void Remote_BuildFrame(uint8_t *f)
{
    static const uint16_t pd_group2[] = {
        GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12,
        GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
    };
    uint8_t i;

    /* 帧头 */
    f[0] = REMOTE_FRAME_HEADER_0;
    f[1] = REMOTE_FRAME_HEADER_1;

    /* 摇杆：ADC_CH14/15 为一组，CH7/11 为另一组（沿用原映射） */
    Remote_PutInt16(&f[2],  Remote_ReadStick(ADC_CHANNEL_14, REMOTE_STICK_CENTER, REMOTE_STICK_DEADZONE));
    Remote_PutInt16(&f[4],  Remote_ReadStick(ADC_CHANNEL_15, REMOTE_STICK_CENTER, REMOTE_STICK_DEADZONE));
    Remote_PutInt16(&f[6],  Remote_ReadStick(ADC_CHANNEL_7,  REMOTE_STICK_CENTER, REMOTE_STICK_DEADZONE));
    Remote_PutInt16(&f[8],  Remote_ReadStick(ADC_CHANNEL_11, REMOTE_STICK_CENTER, REMOTE_STICK_DEADZONE));

    /* 按键：3 字节，共 22 个键，位映射沿用原代码 */
    f[10] = 0U;
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_6)  << 0U);
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_5)  << 1U);
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_4)  << 2U);
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_1)  << 3U);
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_0)  << 4U);
    f[10] |= (uint8_t)(Remote_Key(GPIOC, GPIO_PIN_3)  << 5U);
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_8)  << 6U);
    f[10] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_11) << 7U);

    f[11] = 0U;
    for (i = 0U; i < 6U; ++i)
        f[11] |= (uint8_t)(Remote_Key(GPIOD, pd_group2[i]) << i);
    f[11] |= (uint8_t)(Remote_Key(GPIOA, GPIO_PIN_12) << 6U);
    f[11] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_6)  << 7U);

    f[12] = 0U;
    f[12] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_3) << 0U);
    f[12] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_4) << 1U);
    f[12] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_5) << 2U);
    f[12] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_2) << 3U);
    f[12] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_1) << 4U);
    f[12] |= (uint8_t)(Remote_Key(GPIOE, GPIO_PIN_0) << 5U);
    f[12] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_3) << 6U);
    f[12] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_4) << 7U);

    f[13] = 0U;   /* 预留：CRC8 */
    f[13] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_7) << 0U);
    f[13] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_6) << 1U);
    f[13] |= (uint8_t)(Remote_Key(GPIOD, GPIO_PIN_5) << 2U);

    f[14] = REMOTE_PROTOCOL_VERSION;
    f[15] = REMOTE_FRAME_TAIL;
}

/* ============================================================
 *  DMA 发送队列
 * ============================================================ */

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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart6 || remote_tx_active == 0U) return;
    remote_tx_read = (uint8_t)((remote_tx_read + 1U) % REMOTE_TX_QUEUE_DEPTH);
    remote_tx_count--;
    remote_tx_active = 0U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6 && (huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
        remote_tx_error = 1U;
}

/* ============================================================
 *  用户接口
 * ============================================================ */

void Remote_Init(void)
{
    remote_tx_read   = 0U;
    remote_tx_write  = 0U;
    remote_tx_count  = 0U;
    remote_tx_active = 0U;
    remote_tx_error  = 0U;
    remote_next_tick = HAL_GetTick();
}

void Remote_Loop(void)
{
    uint32_t primask;
    const uint32_t now = HAL_GetTick();

    /* DMA 错误恢复：中止、清空队列 */
    if (remote_tx_error != 0U)
    {
        (void)HAL_UART_AbortTransmit(&huart6);
        primask = __get_PRIMASK();
        __disable_irq();
        remote_tx_read   = remote_tx_write;
        remote_tx_count  = 0U;
        remote_tx_active = 0U;
        remote_tx_error  = 0U;
        __set_PRIMASK(primask);
    }

    Remote_StartTx();

    /* 周期到点才采样并入队 */
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