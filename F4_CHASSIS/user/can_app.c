#include "can_app.h"

#include "board_can.h"
#include "can.h"

static bool CAN_ConfigureFilter(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK;
}

void CAN1_Start(void)
{
    if (!CAN_ConfigureFilter(&hcan1, 0U)
        || (HAL_CAN_Start(&hcan1) != HAL_OK)
        || (HAL_CAN_ActivateNotification(
                &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK))
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void CAN2_Start(void)
{
    if (!CAN_ConfigureFilter(&hcan2, 14U)
        || (HAL_CAN_Start(&hcan2) != HAL_OK)
        || (HAL_CAN_ActivateNotification(
                &hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK))
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};

    if ((hcan == NULL)
        || (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK))
    {
        return;
    }

    if (hcan->Instance == CAN1)
    {
        Board_CAN_RxCallback(&header, data);
    }
}
