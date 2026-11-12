/*
 * @file encoder_link.h
 * @brief encoder_link模块的用户接口与实现。
 */

#ifndef ENCODER_LINK_H
#define ENCODER_LINK_H

#include <stdbool.h>
#include <stdint.h>




#ifndef ENCODER_LINK_FRAME_WORDS
#define ENCODER_LINK_FRAME_WORDS 6U

#endif

#define ENCODER_LINK_FRAME_BYTES (ENCODER_LINK_FRAME_WORDS * 2U)


#ifndef ENCODER_LINK_IDX_C1_H
#define ENCODER_LINK_IDX_C1_H 1U

#endif
#ifndef ENCODER_LINK_IDX_C1_L
#define ENCODER_LINK_IDX_C1_L 2U

#endif
#ifndef ENCODER_LINK_IDX_C2_H
#define ENCODER_LINK_IDX_C2_H 3U

#endif
#ifndef ENCODER_LINK_IDX_C2_L
#define ENCODER_LINK_IDX_C2_L 4U

#endif
#ifndef ENCODER_LINK_IDX_YAW
#define ENCODER_LINK_IDX_YAW 5U

#endif




#ifndef ENCODER_LINK_CS_ENABLE
#define ENCODER_LINK_CS_ENABLE 1

#endif

#ifndef ENCODER_LINK_CS_PORT
#define ENCODER_LINK_CS_PORT GPIOA
#define ENCODER_LINK_CS_PIN GPIO_PIN_15
#define ENCODER_LINK_CS_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#endif


#ifndef ENCODER_LINK_SPI_PRESCALER
#define ENCODER_LINK_SPI_PRESCALER SPI_BAUDRATEPRESCALER_16

#endif

#ifndef ENCODER_LINK_TIMEOUT_MS
#define ENCODER_LINK_TIMEOUT_MS 2U

#endif


#ifndef ENCODER_LINK_RETRY_MS
#define ENCODER_LINK_RETRY_MS 100U

#endif


#ifndef ENCODER_LINK_MAX_DELTA
#define ENCODER_LINK_MAX_DELTA 20000

#endif


#ifndef ENCODER_LINK_OFFLINE_ERRORS
#define ENCODER_LINK_OFFLINE_ERRORS 5U

#endif



typedef struct
{
    int32_t encoder1_count; /* 数据长度或数量。 */
    int32_t encoder2_count; /* 数据长度或数量。 */
    uint16_t yaw_raw; /* 成员变量。 */
    uint32_t frame_count; /* 数据缓冲区或数据帧。 */
    uint32_t error_count; /* 数据长度或数量。 */
    bool online; /* 状态标志。 */
} encoder_link_t;

extern volatile encoder_link_t g_encoder_link; /* 成员变量。 */




/* 功能：执行 EncoderLink_Init 函数对应的模块操作。 */
bool EncoderLink_Init(void);


/* 功能：执行 EncoderLink_Poll 函数对应的模块操作。 */
bool EncoderLink_Poll(float yaw_deg);


/* 功能：执行 EncoderLink_GetYawRaw 函数对应的模块操作。 */
uint16_t EncoderLink_GetYawRaw(void);

#endif
