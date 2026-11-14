/*
 * @file bsp_can.c
 * @brief bsp_can 模块的用户接口与实现。
 */

#include "bsp_can.h"

#include <string.h>

#include "cmsis_os2.h"
#include "fdcan.h"

volatile uint32_t can1_cnt = 0U;
volatile uint32_t can2_cnt = 0U;
volatile uint32_t can3_cnt = 0U;
volatile uint32_t can1_tx_cnt = 0U;
volatile uint32_t can2_tx_cnt = 0U;
volatile uint32_t can3_tx_cnt = 0U;
volatile uint32_t can1_rx_overflow_count = 0U;
volatile uint32_t can2_rx_overflow_count = 0U;
volatile uint32_t can3_rx_overflow_count = 0U;
volatile bsp_can_diagnostics_t can1_diag, can2_diag, can3_diag;

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    IRQn_Type irq;
    uint32_t last_attempt_tick;
} bsp_can_recover_t;

static bsp_can_recover_t can_recover[BSP_CAN_CHANNEL_COUNT] =
{
    {&hfdcan1, FDCAN1_IT0_IRQn, 0U},
    {&hfdcan2, FDCAN2_IT0_IRQn, 0U},
    {&hfdcan3, FDCAN3_IT0_IRQn, 0U},
};
static volatile bsp_can_diagnostics_t *BspCan_Diagnostics(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) return &can1_diag;
    if (hfdcan == &hfdcan2) return &can2_diag;
    if (hfdcan == &hfdcan3) return &can3_diag;
    return NULL;
}

static osMessageQueueId_t can_rx_queue;
/* 功能：处理BspCan_Start对应的输入并更新相关状态。 */
static HAL_StatusTypeDef BspCan_Start(FDCAN_HandleTypeDef *hfdcan                          );

/* 功能：发送BspCan_SendStartupZero对应的数据或控制指令。 */
static void BspCan_SendStartupZero(void)
{
    can_frame_t frame = {0};
    uint8_t repeat;
    frame.hfdcan = &hfdcan2;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.length = 8U;
    for (repeat = 0U; repeat < 2U; repeat++)
    {
        frame.id = 0x200U;
        (void)BspCan_Send(&frame);
        frame.id = 0x1FFU;
        (void)BspCan_Send(&frame);
    }
}

/* 功能：处理BspCan_IsBusOff对应的输入并更新相关状态。 */
static bool BspCan_IsBusOff(const FDCAN_HandleTypeDef *hfdcan)
{
    return (hfdcan != NULL) && (hfdcan->Instance != NULL) && ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U);
}

/* 功能：处理BspCan_RestartChannel对应的输入并更新相关状态。 */
static bool BspCan_RestartChannel(bsp_can_recover_t *channel)
{
    if (channel == NULL)
    {
        return false;
    }
    HAL_NVIC_DisableIRQ(channel->irq);
    (void)HAL_FDCAN_DeInit(channel->handle);

    if (channel->handle == &hfdcan1)
    {
        MX_FDCAN1_Init();
    }
    else if (channel->handle == &hfdcan2)
    {
        MX_FDCAN2_Init();
    }
    else
    {
        MX_FDCAN3_Init();
    }

    if (BspCan_Start(channel->handle) != HAL_OK)
    {
        HAL_NVIC_EnableIRQ(channel->irq);
        return false;
    }
    return true;
}

/* 功能：执行BspCan_Service对应的周期任务并维护运行状态。 */
void BspCan_Service(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t index;

    for (index = 0U; index < BSP_CAN_CHANNEL_COUNT; index++)
    {
        bsp_can_recover_t *channel = &can_recover[index];
        volatile bsp_can_diagnostics_t *diag = BspCan_Diagnostics(channel->handle);

        if ((diag == NULL) || (channel->handle == NULL))
        {
            continue;
        }
        { uint32_t ecr = channel->handle->Instance->ECR; uint32_t tec = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos; uint32_t rec = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos; if (!BspCan_IsBusOff(channel->handle) && tec < BSP_CAN_ERROR_RECOVER_TEC && rec < BSP_CAN_ERROR_RECOVER_REC) { continue; } }
        if ((now - channel->last_attempt_tick) < BSP_CAN_RECOVER_INTERVAL_MS)
        {
            continue;
        }
        channel->last_attempt_tick = now;
        diag->recover_last_tick = now;
        diag->recover_attempts++;

        (void)osKernelLock();
        if (BspCan_RestartChannel(channel))
        {
            diag->recover_success++;
        }
        else
        {
            diag->recover_failed++;
        }
        (void)osKernelUnlock();
    }
}

/* 功能：将BspCan_RxQueueInit对应的目标数据加入执行队列。 */
bool BspCan_RxQueueInit(void)
{
    if (can_rx_queue != NULL)
    {
        return true;
    }
    can_rx_queue = osMessageQueueNew(BSP_CAN_RX_QUEUE_DEPTH, sizeof(can_frame_t), NULL);
    return can_rx_queue != NULL;
}

/* 功能：执行BspCan_RxTask对应的周期任务并维护运行状态。 */
void BspCan_RxTask(void *argument)
{
    can_frame_t frame;
    uint8_t processed_frames;

    (void)argument;
    if (can_rx_queue == NULL)
    {
        for (;;)
        {
            osDelay(1U);
        }
    }
    for (;;)
    {
        processed_frames = 0U;
        if (osMessageQueueGet(can_rx_queue, &frame, NULL, osWaitForever) == osOK)
        {
            BspCan_OnFrame(&frame);
            processed_frames++;
            while (processed_frames < BSP_CAN_RX_MAX_FRAMES_PER_SLICE &&
                   osMessageQueueGet(can_rx_queue, &frame, NULL, 0U) == osOK)
            {
                BspCan_OnFrame(&frame);
                processed_frames++;
            }
            if (processed_frames >= BSP_CAN_RX_MAX_FRAMES_PER_SLICE)
            {
                osThreadYield();
            }
        }
    }
}

/* 功能：初始化BspCan_Init对应的外设、参数和运行状态。 */
HAL_StatusTypeDef BspCan_Init(void)
{
    static bool startup_zero_sent;

    HAL_StatusTypeDef status = HAL_OK;

    if (BspCan_Start(&hfdcan1) != HAL_OK)
    {
        status = HAL_ERROR;
    }
    if (BspCan_Start(&hfdcan2) != HAL_OK)
    {
        status = HAL_ERROR;
    }
    if (BspCan_Start(&hfdcan3) != HAL_OK)
    {
        status = HAL_ERROR;
    }
    if (!startup_zero_sent)
    {
        BspCan_SendStartupZero();
        startup_zero_sent = true;
    }
    return status;
}

/* 功能：处理BspCan_Start对应的输入并更新相关状态。 */
static HAL_StatusTypeDef BspCan_Start(FDCAN_HandleTypeDef *hfdcan                          )
{
    FDCAN_FilterTypeDef filter = {0};

    hfdcan->Init.AutoRetransmission = DISABLE;
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_DAR);

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if ((hfdcan == &hfdcan1) || (hfdcan == &hfdcan2) || (hfdcan == &hfdcan3))
    {
        FDCAN_FilterTypeDef ext_filter = {0};
        ext_filter.IdType = FDCAN_EXTENDED_ID;
        ext_filter.FilterIndex = 0U;
        ext_filter.FilterType = FDCAN_FILTER_RANGE;
        ext_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        ext_filter.FilterID1 = 0U;
        ext_filter.FilterID2 = 0x1FFFFFFFU;
        if (HAL_FDCAN_ConfigFilter(hfdcan, &ext_filter) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) !=
            HAL_OK ||
        HAL_FDCAN_Start(hfdcan) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* 功能：发送BspCan_Send对应的数据或控制指令。 */
bool BspCan_Send(const can_frame_t *frame                               )
{
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_TxHeaderTypeDef header = {0};
    volatile bsp_can_diagnostics_t *diag;

    if ((frame == NULL) || (frame->length > 8U))
    {
        return false;
    }
    hfdcan = frame->hfdcan;
    if ((hfdcan != &hfdcan1) && (hfdcan != &hfdcan2) && (hfdcan != &hfdcan3))
    {
        return false;
    }

    diag = BspCan_Diagnostics(hfdcan);
    if (diag == NULL)
    {
        return false;
    }

    if (BspCan_IsBusOff(hfdcan))
    {

        return false;
    }

    header.Identifier = frame->id;
    header.IdType = frame->id_type;
    if ((header.IdType != FDCAN_STANDARD_ID && header.IdType != FDCAN_EXTENDED_ID) ||
        ((header.IdType == FDCAN_STANDARD_ID) && (header.Identifier > 0x7FFU)) ||
        ((header.IdType == FDCAN_EXTENDED_ID) && (header.Identifier > 0x1FFFFFFFU)))
    {
        return false;
    }
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = frame->length;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    {
        HAL_StatusTypeDef ret;
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
        {
            diag->tx_fifo_full++;
            __set_PRIMASK(primask);
            return false;
        }

        ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)frame->data);
        if (ret == HAL_OK)
        {
            diag->tx_enqueued++;
            if (hfdcan == &hfdcan1) can1_tx_cnt++;
            else if (hfdcan == &hfdcan2) can2_tx_cnt++;
            else if (hfdcan == &hfdcan3) can3_tx_cnt++;
        }
        else
        {
            diag->tx_hal_error++;
            diag->last_tx_hal_error = hfdcan->ErrorCode;
        }
        __set_PRIMASK(primask);
        return ret == HAL_OK;
    }
}

/* 功能：处理BspCan_HasBusOff对应的输入并更新相关状态。 */
bool BspCan_HasBusOff(void)
{

/* 功能：完成BspCan_IsBusOff对应的数据处理。 */
    return BspCan_IsBusOff(&hfdcan1) || BspCan_IsBusOff(&hfdcan2) || BspCan_IsBusOff(&hfdcan3);
}

/* 功能：处理BspCan_Recover对应的输入并更新相关状态。 */
bool BspCan_Recover(void)
{

    BspCan_Service();
    return !BspCan_HasBusOff();
}

/* 功能：处理HAL_FDCAN_RxFifo0Callback对应的输入并更新相关状态。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan                          ,
                               uint32_t rx_fifo0_its                             )
{
    FDCAN_RxHeaderTypeDef header;
    can_frame_t frame;
    uint8_t rx_data[64];
    volatile bsp_can_diagnostics_t *diag = BspCan_Diagnostics(hfdcan);
    if (diag == NULL)
    {
        return;
    }
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
        diag->rx_hw_lost_events++;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        memset(&frame, 0, sizeof(frame));
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, rx_data) != HAL_OK)
        {
            return;
        }

        frame.hfdcan = hfdcan;
        frame.id = header.Identifier;
        frame.id_type = header.IdType;
        frame.length = (uint8_t)header.DataLength;
        if (hfdcan == &hfdcan1)
        {
            can1_cnt++;
        }
        else if (hfdcan == &hfdcan2)
        {
            can2_cnt++;
        }
        else if (hfdcan == &hfdcan3)
        {
            can3_cnt++;
        }
        diag->last_rx_id = frame.id;
        if (frame.id_type == FDCAN_STANDARD_ID) diag->rx_standard++;
        else if (frame.id_type == FDCAN_EXTENDED_ID) diag->rx_extended++;
        if (header.DataLength > FDCAN_DLC_BYTES_8 || header.FDFormat != FDCAN_CLASSIC_CAN ||
            header.RxFrameType != FDCAN_DATA_FRAME)
        {
            diag->rx_invalid++;
            continue;
        }
        memcpy(frame.data, rx_data, frame.length);
        if ((can_rx_queue == NULL) || (osMessageQueuePut(can_rx_queue, &frame, 0U, 0U) != osOK))
        {
            if (hfdcan == &hfdcan1)
            {
                can1_rx_overflow_count++;
            }
            else if (hfdcan == &hfdcan2)
            {
                can2_rx_overflow_count++;
            }
            else if (hfdcan == &hfdcan3)
            {
                can3_rx_overflow_count++;
            }
        }
    }
}
