#include "can_app.h"

#include "ak_motor.h"
#include "board_can.h"
#include "can.h"
#include "my_main.h"

/**
 * @brief CAN1 初始化
 */
HAL_StatusTypeDef CAN1_init(void)
{
    CAN_FilterTypeDef filter = {0};

    /* 配置 CAN1 过滤器 */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank  = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        return HAL_ERROR;

        
        /* 启动 CAN1 */
        if (HAL_CAN_Start(&hcan1) != HAL_OK)
        return HAL_ERROR;
        
        /* 开启 FIFO0 接收中断 */
        if (HAL_CAN_ActivateNotification(
                &hcan1,
                CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
            return HAL_ERROR;
    return HAL_OK;
}

/**
 * @brief CAN2 初始化
 */ 
HAL_StatusTypeDef CAN2_init(void)
{
    CAN_FilterTypeDef filter = {0};


    /* 配置 CAN2 过滤器 */
    filter.FilterBank           = 14;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank  = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
        return HAL_ERROR;

        
        /* 启动 CAN2 */
        if (HAL_CAN_Start(&hcan2) != HAL_OK)
        return HAL_ERROR;
        
        /* 开启 FIFO0 接收中断 */
        if (HAL_CAN_ActivateNotification(
                &hcan2,
                CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
            return HAL_ERROR;
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
            Chassis_rx(&rx_header, rx_data);
        }
        else if (hcan->Instance == CAN2)
        {
            for (uint8_t i = 0U; i < 4U; ++i)
            {
                AkMotor_ParseFeedback(&g_chassis_motors[i],
                                            &rx_header, rx_data);
                
            }
        }
    }
}
