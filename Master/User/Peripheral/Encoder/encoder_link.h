/*
 * @file encoder_link.h
 * @brief 主机与 SPI 从机的码盘链路：接收两路累计计数并更新全场定位。
 *
 * 帧布局参考《机器人位姿详解 V1.3》：
 *   m1 = (SPI1_Buffer_Tx[1] << 16 | SPI1_Buffer_Tx[2]) * PULSE2DIS_ENCODE1;
 *   m2 = (SPI1_Buffer_Tx[3] << 16 | SPI1_Buffer_Tx[4]) * PULSE2DIS_ENCODE2;
 *   GYRO_Ang = SPI1_Buffer_Tx[1 + CODER_NUM * 2];
 * 本模块负责收帧、拼接计数和合理性检查，再交给 Whole_UpdateFromCounts()；
 * 比例换算在 whole.c 中完成。A/B 正交计数、累计和过零展开由从机完成，
 * H723 主机每个周期读取一次累计值。
 */

#ifndef ENCODER_LINK_H
#define ENCODER_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* 帧格式配置。 */

/* 每帧的 16 位字数量：字 0 为帧头或命令，两路计数各占两个字，字 5 起为陀螺数据。 */
#ifndef ENCODER_LINK_FRAME_WORDS
#define ENCODER_LINK_FRAME_WORDS 6U

#endif

#define ENCODER_LINK_FRAME_BYTES (ENCODER_LINK_FRAME_WORDS * 2U)

/* 各字段在帧中的字下标，与参考主机的 SPI1_Buffer_Tx[] 一致。 */
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

/* 硬件配置。 */

/* 板上未配置专用片选，默认使用 PA15 作为软件片选。
 * 若从机片选直接接地或改接其他引脚，可修改下方配置宏；
 * 将 ENCODER_LINK_CS_ENABLE 设为 0 可关闭片选操作。 */
#ifndef ENCODER_LINK_CS_ENABLE
#define ENCODER_LINK_CS_ENABLE 1

#endif

#ifndef ENCODER_LINK_CS_PORT
#define ENCODER_LINK_CS_PORT GPIOA
#define ENCODER_LINK_CS_PIN GPIO_PIN_15
#define ENCODER_LINK_CS_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#endif

/* SPI6 时钟分频：将 CubeMX 配置的二分频（约 50 兆位/秒）降为十六分频。 */
#ifndef ENCODER_LINK_SPI_PRESCALER
#define ENCODER_LINK_SPI_PRESCALER SPI_BAUDRATEPRESCALER_16

#endif

#ifndef ENCODER_LINK_TIMEOUT_MS
#define ENCODER_LINK_TIMEOUT_MS 2U

#endif

/* 离线时每 100 毫秒重试一次，避免从机未连接时阻塞控制周期。 */
#ifndef ENCODER_LINK_RETRY_MS
#define ENCODER_LINK_RETRY_MS 100U

#endif

/* 单帧计数增量上限，单位为计数/帧；超限判为坏帧，避免错误数据导致位置跳变。 */
#ifndef ENCODER_LINK_MAX_DELTA
#define ENCODER_LINK_MAX_DELTA 20000

#endif

/* 连续坏帧达到此数量后判定为离线。 */
#ifndef ENCODER_LINK_OFFLINE_ERRORS
#define ENCODER_LINK_OFFLINE_ERRORS 5U

#endif

/* 数据结构。 */

typedef struct
{
    int32_t encoder1_count; /* 从机发送的码盘 1 累计计数。 */
    int32_t encoder2_count; /* 码盘 2 累计计数。 */
    uint16_t yaw_raw;       /* 帧中的陀螺原始字，暂未换算。 */
    uint32_t frame_count;   /* 成功接收的帧数。 */
    uint32_t error_count;   /* 坏帧或超时次数。 */
    bool online;            /* 是否在线。 */
} encoder_link_t;

extern volatile encoder_link_t g_encoder_link;

/* 接口声明。 */

/* 初始化 SPI6 的 8 位数据宽度和片选引脚；返回 false 表示 SPI 初始化失败。 */
bool EncoderLink_Init(void);

/* 在周期任务中读取一帧并更新全场位置，例如以 500 赫兹调用。
 * 解析两路累计计数后调用 Whole_UpdateFromCounts(c1, c2, yaw_deg)。
 * 返回 true 表示本周期接收到有效帧。 */
bool EncoderLink_Poll(float yaw_deg);

/* 获取最近一帧的陀螺原始字，供后续航向解算使用。 */
uint16_t EncoderLink_GetYawRaw(void);

#endif /* 头文件保护结束。 */
