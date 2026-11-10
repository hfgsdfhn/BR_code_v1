#include "bsp_can.h"
#include <stm32f4xx_hal_can.h>

void BSP_CAN_Init(bsp_can_t *self, can_type_t can_type)
{
    if (!self || !self->handle) return;

    self->can_type = can_type;    

#if use_can
    FDCAN_FilterTypeDef filter = {0};
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;

    /* 按这路实际使用的帧格式只配一条 filter */
    filter.IdType = (can_type == ext)
                    ? FDCAN_EXTENDED_ID
                    : FDCAN_STANDARD_ID;
    HAL_FDCAN_ConfigFilter(self->handle, &filter);

    HAL_FDCAN_Start(self->handle);
    HAL_FDCAN_ActivateNotification(self->handle,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#else
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(self->handle, &filter);
    HAL_CAN_Start(self->handle);
    HAL_CAN_ActivateNotification(self->handle, CAN_IT_RX_FIFO0_MSG_PENDING);
#endif
}

/* ============ 发送 ============ */
bool BSP_CAN_Send(bsp_can_t *self, BSP_CAN_Frame_t *frame)
{
    if (!self || !self->handle || !frame || frame->dlc > 8) return false;

#if use_can
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier    = frame->id;
    tx.IdType        = frame->ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType   = FDCAN_DATA_FRAME;
    tx.DataLength    = frame->dlc;
    tx.FDFormat      = FDCAN_CLASSIC_CAN;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    return HAL_FDCAN_AddMessageToTxFifoQ(self->handle, &tx,
                                         frame->data) == HAL_OK;
#else
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;
    if (frame->ext) { tx.ExtId = frame->id; tx.IDE = CAN_ID_EXT; }
    else            { tx.StdId = frame->id; tx.IDE = CAN_ID_STD; }
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = frame->dlc;
    tx.TransmitGlobalTime = DISABLE;

    uint32_t timeout = 0xFFFFF;
    while (HAL_CAN_GetTxMailboxesFreeLevel(self->handle) == 0) {
        if (--timeout == 0) return false;
    }
    return HAL_CAN_AddTxMessage(self->handle, &tx, frame->data,
                                &mailbox) == HAL_OK;
#endif
}

/* ============ 接收中断回调 ============ */

#define BSP_CAN_MAX_INST 4
static bsp_can_t *s_cans[BSP_CAN_MAX_INST];

void BSP_CAN_Bind(bsp_can_t *can)
{
    if (!can) return;
    for (int i = 0; i < BSP_CAN_MAX_INST; i++) {
        if (s_cans[i] == can) return;          // 已注册
        if (!s_cans[i]) { s_cans[i] = can; return; }
    }
    // 满了：可断言/日志
}

static bsp_can_t *find_by_handle(void *h)
{
    for (int i = 0; i < BSP_CAN_MAX_INST; i++)
        if (s_cans[i] && s_cans[i]->handle == h)
            return s_cans[i];
    return NULL;
}

#if use_can
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t its)
{
    if (!(its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;
    bsp_can_t *self = find_by_handle(hfdcan);
    if (!self) return;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8] = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
        return;

    BSP_CAN_Frame_t f = {0};
    f.id  = rx.Identifier;
    f.ext = (rx.IdType == FDCAN_EXTENDED_ID);
    f.dlc = rx.DataLength;
    for (uint8_t i = 0; i < f.dlc && i < 8; i++) f.data[i] = data[i];

    if (self->on_rx) self->on_rx(self, &f);
}
#else
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    bsp_can_t *self = find_by_handle(hcan);
    if (!self) return;

    CAN_RxHeaderTypeDef rx;
    uint8_t data[8] = {0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        return;

    BSP_CAN_Frame_t f = {0};
    f.id  = (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId;
    f.ext = (rx.IDE == CAN_ID_EXT);
    f.dlc = rx.DLC;
    for (uint8_t i = 0; i < f.dlc && i < 8; i++) f.data[i] = data[i];

    if (self->on_rx) self->on_rx(self, &f);
}
#endif