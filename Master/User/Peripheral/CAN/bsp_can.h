/*
 * @file bsp_can.h
 * @brief bsp_can 模块的用户接口与实现。
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "fdcan.h"

#define BSP_CAN_RX_QUEUE_DEPTH 64U
#define BSP_CAN_RX_MAX_FRAMES_PER_SLICE 8U

#define BSP_CAN_RECOVER_INTERVAL_MS 10U
#define BSP_CAN_CHANNEL_COUNT 3U
#define BSP_CAN_ERROR_RECOVER_TEC 200U
#define BSP_CAN_ERROR_RECOVER_REC 120U

extern volatile uint32_t can1_cnt;
extern volatile uint32_t can2_cnt;
extern volatile uint32_t can3_cnt;
extern volatile uint32_t can1_tx_cnt;
extern volatile uint32_t can2_tx_cnt;
extern volatile uint32_t can3_tx_cnt;
extern volatile uint32_t can1_rx_overflow_count;
extern volatile uint32_t can2_rx_overflow_count;
extern volatile uint32_t can3_rx_overflow_count;

typedef struct
{
    uint32_t tx_enqueued;
    uint32_t tx_fifo_full;
    uint32_t tx_hal_error;
    uint32_t last_tx_hal_error;
    uint32_t bus_off_count;
    uint32_t rx_standard;
    uint32_t rx_extended;
    uint32_t rx_hw_lost_events;
    uint32_t rx_invalid;
    uint32_t last_rx_id;

    uint32_t recover_success;
    uint32_t recover_attempts;
    uint32_t recover_failed;
    uint32_t recover_last_tick;
} bsp_can_diagnostics_t;
extern volatile bsp_can_diagnostics_t can1_diag, can2_diag, can3_diag;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t id;
    uint32_t id_type;
    uint8_t length;
    uint8_t data[8];
} can_frame_t;

/* 功能：初始化BspCan_Init对应的外设、参数和运行状态。 */
HAL_StatusTypeDef BspCan_Init(void);

/* 功能：将BspCan_RxQueueInit对应的目标数据加入执行队列。 */
bool BspCan_RxQueueInit(void);

/* 功能：执行BspCan_RxTask对应的周期任务并维护运行状态。 */
void BspCan_RxTask(void *argument);

/* 功能：接收并解析BspCan_OnFrame对应的输入数据，更新模块状态。 */
void BspCan_OnFrame(const can_frame_t *frame                    );

/* 功能：发送BspCan_Send对应的数据或控制指令。 */
bool BspCan_Send(const can_frame_t *frame                                      );

/* 功能：处理BspCan_HasBusOff对应的输入并更新相关状态。 */
bool BspCan_HasBusOff(void);

/* 功能：处理BspCan_Recover对应的输入并更新相关状态。 */
bool BspCan_Recover(void);

/* 功能：执行BspCan_Service对应的周期任务并维护运行状态。 */
void BspCan_Service(void);

#endif
