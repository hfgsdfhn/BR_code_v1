/*
 * @file bsp_can.c
 * @brief 实现三路 FDCAN 的公共收发部分。
 */

/* FDCAN 底层适配：完成三路 CAN 初始化、收发和总线故障恢复。 */
#include "bsp_can.h"

#include <string.h>

#include "cmsis_os2.h"
#include "fdcan.h"

volatile uint32_t can1_cnt = 0U;
volatile uint32_t can2_cnt = 0U;
volatile uint32_t can3_cnt = 0U;
volatile uint32_t can1_rx_overflow_count = 0U;
volatile uint32_t can2_rx_overflow_count = 0U;
volatile uint32_t can3_rx_overflow_count = 0U;
static osMessageQueueId_t can_rx_queue;
/* BspCan_Start：执行该接口对应的控制或数据处理流程。 */
static HAL_StatusTypeDef BspCan_Start(FDCAN_HandleTypeDef *hfdcan /* 待配置并启动的 FDCAN 外设句柄。 */);

/* 向各路 CAN 发送启动阶段的零值帧，清除残留控制量。 */
static void BspCan_SendStartupZero(void)
{
    can_frame_t frame = {0};
    uint8_t repeat;
    frame.id_type = FDCAN_STANDARD_ID;
    frame.length = 8U;
    for (repeat = 0U; repeat < 2U; repeat++)
    {
        frame.hfdcan = &hfdcan2;
        frame.id = 0x200U;
        (void)BspCan_Send(&frame);
        frame.id = 0x1FFU;
        (void)BspCan_Send(&frame);
        frame.hfdcan = &hfdcan3;
        frame.id = 0x200U;
        (void)BspCan_Send(&frame);
    }
}

/* 检查指定 FDCAN 外设是否进入总线关闭状态。 */
static bool BspCan_IsBusOff(const FDCAN_HandleTypeDef *hfdcan)
{
    return (hfdcan != NULL) && (hfdcan->Instance != NULL) && ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U);
}

bool BspCan_RxQueueInit(void)
{
    if (can_rx_queue != NULL)
    {
        return true;
    }
    can_rx_queue = osMessageQueueNew(BSP_CAN_RX_QUEUE_DEPTH, sizeof(can_frame_t), NULL);
    return can_rx_queue != NULL;
}

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

/* BspCan_Start：执行该接口对应的控制或数据处理流程。 */

/* 初始化所有已启用的 FDCAN 外设及接收过滤器。 */
HAL_StatusTypeDef BspCan_Init(void)
{
    static bool startup_zero_sent;

    if (BspCan_Start(&hfdcan1) != HAL_OK || BspCan_Start(&hfdcan2) != HAL_OK || BspCan_Start(&hfdcan3) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (!startup_zero_sent)
    {
        BspCan_SendStartupZero();
        startup_zero_sent = true;
    }
    return HAL_OK;
}

/* 配置并启动单路 FDCAN 外设。 */
static HAL_StatusTypeDef BspCan_Start(FDCAN_HandleTypeDef *hfdcan /* 待配置并启动的 FDCAN 外设句柄。 */)
{
    FDCAN_FilterTypeDef filter = {0}; /* 当前 FDCAN 总线使用的标准帧接收过滤器配置。 */

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
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* 将统一 CAN 帧转换为 HAL 格式并发送到指定总线。 */
bool BspCan_Send(const can_frame_t *frame /* 包含目标总线及待发送内容的经典 CAN 数据帧。 */)
{
    FDCAN_HandleTypeDef *hfdcan;        /* 待发送帧使用的 FDCAN 外设句柄。 */
    FDCAN_TxHeaderTypeDef header = {0}; /* 本次 CAN 发送使用的帧头配置。 */

    if ((frame == NULL) || (frame->length > 8U))
    {
        return false;
    }
    hfdcan = frame->hfdcan;
    if ((hfdcan != &hfdcan1) && (hfdcan != &hfdcan2) && (hfdcan != &hfdcan3))
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
        /* 保护 FIFO 选择和入队操作，防止任务与中断并发发送时发生冲突。 */
        __disable_irq();
        if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
        {
            __set_PRIMASK(primask);
            return false;
        }

        ret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)frame->data);
        __set_PRIMASK(primask);
        return ret == HAL_OK;
    }
}

/* 查询任意一路 CAN 是否发生过总线关闭。 */
bool BspCan_HasBusOff(void)
{
    return BspCan_IsBusOff(&hfdcan1) || BspCan_IsBusOff(&hfdcan2) || BspCan_IsBusOff(&hfdcan3);
}

/* 清除错误标志并重新启动发生总线关闭的 CAN 外设。 */
bool BspCan_Recover(void)
{
    if (!BspCan_HasBusOff())
    {
        return true;
    }

    if (BspCan_IsBusOff(&hfdcan1))
    {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        (void)HAL_FDCAN_DeInit(&hfdcan1);
        MX_FDCAN1_Init();
        if (BspCan_Start(&hfdcan1) != HAL_OK)
        {
            return false;
        }
    }
    if (BspCan_IsBusOff(&hfdcan2))
    {
        (void)HAL_FDCAN_Stop(&hfdcan2);
        (void)HAL_FDCAN_DeInit(&hfdcan2);
        MX_FDCAN2_Init();
        if (BspCan_Start(&hfdcan2) != HAL_OK)
        {
            return false;
        }
    }
    if (BspCan_IsBusOff(&hfdcan3))
    {
        (void)HAL_FDCAN_Stop(&hfdcan3);
        (void)HAL_FDCAN_DeInit(&hfdcan3);
        MX_FDCAN3_Init();
        if (BspCan_Start(&hfdcan3) != HAL_OK)
        {
            return false;
        }
    }
    return !BspCan_HasBusOff();
}

/* 功能：读取 FIFO 中的 CAN 帧并交给电机模块。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan /* 触发接收中断的 FDCAN 外设句柄。 */,
                               uint32_t rx_fifo0_its /* FDCAN 接收 FIFO0 的中断状态位。 */)
{
    FDCAN_RxHeaderTypeDef header; /* 从接收 FIFO 读取的 FDCAN 帧头。 */
    can_frame_t frame;            /* 交给上层回调处理的经典 CAN 数据帧。 */
    (void)rx_fifo0_its;
    if (hfdcan == NULL)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        memset(&frame, 0, sizeof(frame));
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK)
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
