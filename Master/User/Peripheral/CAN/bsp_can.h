





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

/* Counters are cumulative. tx_enqueued is not a motor acknowledgement. */
typedef struct
{
    uint32_t tx_enqueued;
    uint32_t tx_fifo_full;
    uint32_t tx_hal_error;
    uint32_t last_tx_hal_error;
    uint32_t rx_standard;
    uint32_t rx_extended;
    uint32_t rx_hw_lost_events;
    uint32_t rx_invalid;
    uint32_t last_rx_id;
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


HAL_StatusTypeDef BspCan_Init(void);


bool BspCan_RxQueueInit(void);


void BspCan_RxTask(void *argument);


void BspCan_OnFrame(const can_frame_t *frame                    );


bool BspCan_Send(const can_frame_t *frame                                      );


bool BspCan_HasBusOff(void);


bool BspCan_Recover(void);

#endif
