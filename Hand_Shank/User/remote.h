/*
 * @file remote.h
 * @brief remote 模块的用户接口。
 */
#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"   /* 按你的工程实际头文件调整，需包含 HAL 与 huart6/hadc1 声明 */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 帧协议定义 ---------------- */
#define REMOTE_FRAME_SIZE        16U
#define REMOTE_FRAME_HEADER_0    0xAAU
#define REMOTE_FRAME_HEADER_1    0x55U
#define REMOTE_FRAME_TAIL        0x5AU
#define REMOTE_PROTOCOL_VERSION  0x02U

/* 摇杆定点范围：-1000 ~ +1000 对应 -1.0 ~ +1.0 */
#define REMOTE_STICK_FULL_SCALE  1000

/* 摇杆中心与死区（ADC 12 位，0~4095） */
#define REMOTE_STICK_CENTER      2048U
#define REMOTE_STICK_DEADZONE    100U

/* 发送周期与队列深度 */
#define REMOTE_TX_PERIOD_MS      20U
#define REMOTE_TX_QUEUE_DEPTH    4U

/* ---------------- 用户接口 ---------------- */
void Remote_Init(void);
void Remote_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_H */