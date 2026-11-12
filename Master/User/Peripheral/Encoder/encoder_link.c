/*
 * @file encoder_link.c
 * @brief encoder_link模块的用户接口与实现。
 */

#include "encoder_link.h"
#include "down.h"
#include "spi.h"
#include "main.h"

#include <string.h>

volatile encoder_link_t g_encoder_link;

static uint16_t s_words[ENCODER_LINK_FRAME_WORDS];
static uint32_t s_last_attempt_ms;
static uint32_t s_consecutive_errors;
static bool s_attempted;
static bool s_ready;


/* 功能：执行 EncoderLink_Cs 函数对应的模块操作。 */
static void EncoderLink_Cs(GPIO_PinState level)
{
#if ENCODER_LINK_CS_ENABLE
    HAL_GPIO_WritePin(ENCODER_LINK_CS_PORT, ENCODER_LINK_CS_PIN, level);
#else
    (void)level;
#endif

}


/* 功能：执行 EncoderLink_Init 函数对应的模块操作。 */
bool EncoderLink_Init(void)
{
#if ENCODER_LINK_CS_ENABLE
    GPIO_InitTypeDef gpio = {0};

    ENCODER_LINK_CS_CLK_ENABLE();
    gpio.Pin = ENCODER_LINK_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ENCODER_LINK_CS_PORT, &gpio);
    EncoderLink_Cs(GPIO_PIN_SET);
#endif


    hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi6.Init.BaudRatePrescaler = ENCODER_LINK_SPI_PRESCALER;
    if (HAL_SPI_Init(&hspi6) != HAL_OK)
    {
        s_ready = false;
        return false;
    }

    g_encoder_link.encoder1_count = 0;
    g_encoder_link.encoder2_count = 0;
    g_encoder_link.yaw_raw = 0U;
    g_encoder_link.frame_count = 0U;
    g_encoder_link.error_count = 0U;
    g_encoder_link.online = false;
    s_consecutive_errors = 0U;
    s_last_attempt_ms = 0U;
    s_attempted = false;
    s_ready = true;

    return true;
}


/* 功能：执行 EncoderLink_RecordError 函数对应的模块操作。 */
static void EncoderLink_RecordError(void)
{
    g_encoder_link.error_count++;
    s_consecutive_errors++;
    if (s_consecutive_errors >= ENCODER_LINK_OFFLINE_ERRORS)
    {
        g_encoder_link.online = false;
    }
}


/* 功能：执行 EncoderLink_Poll 函数对应的模块操作。 */
bool EncoderLink_Poll(float yaw_deg)
{
    uint8_t tx[ENCODER_LINK_FRAME_BYTES];
    uint8_t rx[ENCODER_LINK_FRAME_BYTES];
    HAL_StatusTypeDef status;
    uint32_t now;
    uint32_t i;
    int32_t c1;
    int32_t c2;

    if (!s_ready)
    {
        return false;
    }


    now = HAL_GetTick();
    if ((!g_encoder_link.online) && s_attempted && ((now - s_last_attempt_ms) < ENCODER_LINK_RETRY_MS))
    {
        return false;
    }
    s_last_attempt_ms = now;
    s_attempted = true;

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    EncoderLink_Cs(GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi6, tx, rx, (uint16_t)ENCODER_LINK_FRAME_BYTES, ENCODER_LINK_TIMEOUT_MS);
    EncoderLink_Cs(GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        EncoderLink_RecordError();
        return false;
    }


    {
        bool all_ff = true;

        for (i = 0U; i < ENCODER_LINK_FRAME_BYTES; i++)
        {
            if (rx[i] != 0xFFU)
            {
                all_ff = false;
                break;
            }
        }
        if (all_ff)
        {
            EncoderLink_RecordError();
            return false;
        }
    }


    for (i = 0U; i < ENCODER_LINK_FRAME_WORDS; i++)
    {
        s_words[i] = (uint16_t)(((uint16_t)rx[2U * i] << 8) | (uint16_t)rx[2U * i + 1U]);
    }

    c1 = (int32_t)(((uint32_t)s_words[ENCODER_LINK_IDX_C1_H] << 16) | (uint32_t)s_words[ENCODER_LINK_IDX_C1_L]);
    c2 = (int32_t)(((uint32_t)s_words[ENCODER_LINK_IDX_C2_H] << 16) | (uint32_t)s_words[ENCODER_LINK_IDX_C2_L]);


    if (g_encoder_link.online)
    {
        int32_t d1 = (int32_t)((uint32_t)c1 - (uint32_t)g_encoder_link.encoder1_count);
        int32_t d2 = (int32_t)((uint32_t)c2 - (uint32_t)g_encoder_link.encoder2_count);

        if ((d1 > ENCODER_LINK_MAX_DELTA) || (d1 < -ENCODER_LINK_MAX_DELTA) || (d2 > ENCODER_LINK_MAX_DELTA) ||
            (d2 < -ENCODER_LINK_MAX_DELTA))
        {
            EncoderLink_RecordError();
            return false;
        }
    }

    g_encoder_link.encoder1_count = c1;
    g_encoder_link.encoder2_count = c2;
    g_encoder_link.yaw_raw = s_words[ENCODER_LINK_IDX_YAW];
    g_encoder_link.frame_count++;

    if (!g_encoder_link.online)
    {
        const whole_position_t pose = Whole_GetPosition();
        Whole_Reset(pose.x_mm, pose.y_mm, yaw_deg);
    }
    g_encoder_link.online = true;
    s_consecutive_errors = 0U;
    Whole_UpdateFromCounts(c1, c2, yaw_deg);

    return true;
}


/* 功能：执行 EncoderLink_GetYawRaw 函数对应的模块操作。 */
uint16_t EncoderLink_GetYawRaw(void)
{
    return g_encoder_link.yaw_raw;
}
