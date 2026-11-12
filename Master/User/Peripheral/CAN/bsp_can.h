/*
 * @file bsp_can.h
 * @brief bsp_can模块的用户接口与实现。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

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

/* Counters are cumulative. tx_enqueued is not a 电机 acknowledgement. */
typedef struct
{
    uint32_t tx_enqueued; /* 成员变量。 */
    uint32_t tx_fifo_full; /* 成员变量。 */
    uint32_t tx_hal_error; /* 成员变量。 */
    uint32_t last_tx_hal_error; /* 成员变量。 */
    uint32_t rx_standard; /* 成员变量。 */
    uint32_t rx_extended; /* 成员变量。 */
    uint32_t rx_hw_lost_events; /* 成员变量。 */
    uint32_t rx_invalid; /* 状态标志。 */
    uint32_t last_rx_id; /* 成员变量。 */
} bsp_can_diagnostics_t;
extern volatile bsp_can_diagnostics_t can1_diag, can2_diag, can3_diag; /* 成员变量。 */

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan; /* 成员变量。 */
    uint32_t id; /* 成员变量。 */
    uint32_t id_type; /* 成员变量。 */
    uint8_t length; /* 数据长度或数量。 */
    uint8_t data[8]; /* 数据缓冲区或数据帧。 */
} can_frame_t;


/* 功能：初始化模块并准备外设资源。 */
HAL_StatusTypeDef BspCan_Init(void);


/* 功能：初始化模块并准备外设资源。 */
bool BspCan_RxQueueInit(void);


/* 功能：执行周期任务主流程。 */
void BspCan_RxTask(void *argument);


/* 功能：接收并解析一帧通信数据。 */
void BspCan_OnFrame(const can_frame_t *frame                    );


/* 功能：发送数据或控制指令。 */
bool BspCan_Send(const can_frame_t *frame                                      );


/* 功能：执行 BspCan_HasBusOff 模块操作并维护相关状态。 */
bool BspCan_HasBusOff(void);


/* 功能：执行通信或设备故障恢复。 */
bool BspCan_Recover(void);

#endif
