#include "board_can.h"

static HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                                      const uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if ((hfdcan == NULL) || (data == NULL) || (length > 8U)) return HAL_ERROR;
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)length << 16;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data);
}

HAL_StatusTypeDef Chassis_SendSpeed(FDCAN_HandleTypeDef *hfdcan,
                                    const chassis_speed_t *speed)
{
    int16_t raw_vx, raw_vy, raw_vw;
    uint8_t tx_data[8] = {0};
    if ((hfdcan == NULL) || (speed == NULL)) return HAL_ERROR;
    raw_vx = (int16_t)(speed->vx * 1000.0f);
    raw_vy = (int16_t)(speed->vy * 1000.0f);
    raw_vw = (int16_t)(speed->vw * 1000.0f);
    tx_data[0] = (uint8_t)raw_vx;
    tx_data[1] = (uint8_t)((uint16_t)raw_vx >> 8);
    tx_data[2] = (uint8_t)raw_vy;
    tx_data[3] = (uint8_t)((uint16_t)raw_vy >> 8);
    tx_data[4] = (uint8_t)raw_vw;
    tx_data[5] = (uint8_t)((uint16_t)raw_vw >> 8);
    return CAN_Transmit(hfdcan, BOARD_ID_SPEED, tx_data, 8U);
}
