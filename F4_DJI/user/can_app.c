#include "can_app.h"

#include "board_can.h"
#include "can.h"
#include "my_main.h"

HAL_StatusTypeDef CAN1_init(void)
{
    CAN_FilterTypeDef filter = {0};

    MX_CAN1_Init();

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN2_init(void)
{
    CAN_FilterTypeDef filter = {0};

    MX_CAN2_Init();

    filter.FilterBank = 14U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_ActivateNotification(&hcan2,
                                     CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan == NULL)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                 &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if (hcan->Instance == CAN1)
        {
            Board_DJI_Rx(&rx_header, rx_data);
        }
        else if (hcan->Instance == CAN2)
        {
            DJI_OnData(&rx_header, rx_data);
        }
    }
}
