#include "board_can.h"

static HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t Header, uint32_t Id, uint8_t *data,uint8_t len)
{
	static FDCAN_TxHeaderTypeDef  tx_Header;
	HAL_StatusTypeDef Sta;
	//处理帧头
	if(len > 8)
	{
		len = 8;
	}
	switch(Header)
	{
		case 0:                                //标准帧
			tx_Header.Identifier = Id;
			tx_Header.IdType = FDCAN_STANDARD_ID;
			tx_Header.TxFrameType = FDCAN_DATA_FRAME;
			tx_Header.DataLength  = len;
			tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			tx_Header.BitRateSwitch = FDCAN_BRS_OFF;
			tx_Header.FDFormat = FDCAN_CLASSIC_CAN;
			tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			tx_Header.MessageMarker = 0;
			break;
		case 1:                               //扩展帧
			tx_Header.Identifier = Id;
			tx_Header.IdType = FDCAN_EXTENDED_ID;
			tx_Header.TxFrameType = FDCAN_DATA_FRAME;
			tx_Header.DataLength  = len;
			tx_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			tx_Header.BitRateSwitch = FDCAN_BRS_OFF;
			tx_Header.FDFormat = FDCAN_CLASSIC_CAN;
			tx_Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
			tx_Header.MessageMarker = 0;
			break;
		default:
			break;	
	}
	//处理数据
	uint8_t trans_data[len];
	for(int i = 0; i < len; i++)
	{
		trans_data[i] = data[i];
	}
	//can发送
	Sta = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_Header,trans_data);
	return Sta;
}

/**
 * @brief 底盘速度 CAN 发送
 * @param hfdcan FDCAN 句柄
 * @param speed 底盘目标速度
 * @return HAL_OK 发送成功，其他值发送失败
 */
HAL_StatusTypeDef Chassis_SendSpeed(FDCAN_HandleTypeDef *hfdcan,
                                    const chassis_speed_t *speed)
{
    int16_t raw_vx;
    int16_t raw_vy;
    int16_t raw_vw;

    uint8_t tx_data[8] = {0};

    if ((hfdcan == NULL) || (speed == NULL))
    {
        return HAL_ERROR;
    }

    /* 浮点速度转换为 int16_t */
    raw_vx = (int16_t)(speed->vx * 1000.0f);
    raw_vy = (int16_t)(speed->vy * 1000.0f);
    raw_vw = (int16_t)(speed->vw * 1000.0f);

    /* vx */
    tx_data[0] = (uint8_t)(raw_vx & 0xFF);
    tx_data[1] = (uint8_t)((uint16_t)raw_vx >> 8);

    /* vy */
    tx_data[2] = (uint8_t)(raw_vy & 0xFF);
    tx_data[3] = (uint8_t)((uint16_t)raw_vy >> 8);

    /* vw */
    tx_data[4] = (uint8_t)(raw_vw & 0xFF);
    tx_data[5] = (uint8_t)((uint16_t)raw_vw >> 8);

    /* 发送标准 CAN 数据帧 */
    return CAN_Transmit(hfdcan,
                        0U,
                        BOARD_ID_SPEED,
                        tx_data,
                        8U);
}
