/*
 * @file bsp_can.h
 * @brief 声明三路 FDCAN 的初始化、发送和接收接口。
 */

/* CAN 帧格式和底层收发接口声明。 */
#ifndef BSP_CAN_H
#define BSP_CAN_H /* CAN 底层头文件保护宏。 */

#include <stdbool.h>
#include <stdint.h>
#include "fdcan.h"

#define BSP_CAN_RX_QUEUE_DEPTH 64U
#define BSP_CAN_RX_MAX_FRAMES_PER_SLICE 8U

extern volatile uint32_t can1_cnt;
extern volatile uint32_t can2_cnt;
extern volatile uint32_t can3_cnt;
extern volatile uint32_t can1_rx_overflow_count;
extern volatile uint32_t can2_rx_overflow_count;
extern volatile uint32_t can3_rx_overflow_count;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan; /* 接收或发送该帧使用的 FDCAN 外设句柄。 */
    uint32_t id;                 /* CAN 帧标识符。 */
    uint32_t id_type;            /* 标识符类型：标准帧或扩展帧。 */
    uint8_t length;              /* 数据段的有效字节数，范围为 0 至 8。 */
    uint8_t data[8];             /* 经典 CAN 帧的 0 至 8 字节数据段。 */
} can_frame_t;                   /* 保存板内各模块统一传递的经典 CAN 数据帧。 */

/* 功能：启动三路 FDCAN 并打开接收中断；初始化并启动全部 FDCAN 外设及接收过滤器。 */
HAL_StatusTypeDef BspCan_Init(void);

bool BspCan_RxQueueInit(void);

void BspCan_RxTask(void *argument);

/* 应用层接收回调，由 BspCan_RxTask 调用。 */
void BspCan_OnFrame(const can_frame_t *frame /* 待分发的 CAN 反馈帧。 */);

/* 功能：通过指定 FDCAN 外设发送一帧数据。 */
bool BspCan_Send(const can_frame_t *frame /* 包含目标 FDCAN 句柄及待发送内容的经典 CAN 数据帧。 */);

/* 功能：检查三路 FDCAN 是否有总线关闭故障；检查是否有任一路 CAN 总线进入 总线关闭 状态。 */
bool BspCan_HasBusOff(void);

/* 功能：分别恢复发生总线关闭的 FDCAN，正常总线保持运行；重新初始化发生 总线关闭 的 CAN 外设。 */
bool BspCan_Recover(void);

#endif
