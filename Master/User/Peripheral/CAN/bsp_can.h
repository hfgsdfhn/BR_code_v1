/*
 * @file bsp_can.h
 * @brief bsp_can模块的用户接口与实现�?
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "fdcan.h"

#define BSP_CAN_RX_QUEUE_DEPTH 64U
#define BSP_CAN_RX_MAX_FRAMES_PER_SLICE 8U

       /* 模块实现细节�?*/
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

/* Counters are cumulative. tx_enqueued is not a 鐢垫満 acknowledgement. */
typedef struct
{
    uint32_t tx_enqueued; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t tx_fifo_full; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t tx_hal_error; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t last_tx_hal_error; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t bus_off_count; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t rx_standard; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t rx_extended; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t rx_hw_lost_events; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t rx_invalid; /* 鐘舵€佹爣蹇椼€?*/
    uint32_t last_rx_id; /* 鎴愬憳鍙橀噺銆?*/
       /* 模块实现细节�?*/
    uint32_t recover_success; /* 鎭㈠鎴愬姛娆℃暟銆?*/
    uint32_t recover_attempts; /* 恢复尝试次数�?*/
    uint32_t recover_failed; /* 鎭㈠澶辫触娆℃暟銆?*/
    uint32_t recover_last_tick; /* 鏈€杩戜竴娆″皾璇曟仮澶嶇殑鏃跺�?ms)�?*/
} bsp_can_diagnostics_t;
extern volatile bsp_can_diagnostics_t can1_diag, can2_diag, can3_diag; /* 鎴愬憳鍙橀噺銆?*/

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t id; /* 鎴愬憳鍙橀噺銆?*/
    uint32_t id_type; /* 鎴愬憳鍙橀噺銆?*/
    uint8_t length; /* 鏁版嵁闀垮害鎴栨暟閲忋€?*/
    uint8_t data[8]; /* 鏁版嵁缂撳啿鍖烘垨鏁版嵁甯с€?*/
} can_frame_t;


/* 模块实现细节�?*/
HAL_StatusTypeDef BspCan_Init(void);


/* 模块实现细节�?*/
bool BspCan_RxQueueInit(void);


/* 模块实现细节�?*/
void BspCan_RxTask(void *argument);


/* 模块实现细节�?*/
void BspCan_OnFrame(const can_frame_t *frame                    );


/* 模块实现细节�?*/
bool BspCan_Send(const can_frame_t *frame                                      );


/* 模块实现细节�?*/
bool BspCan_HasBusOff(void);


/* 模块实现细节�?*/
bool BspCan_Recover(void);


/* 模块实现细节�?*/
void BspCan_Service(void);

#endif
