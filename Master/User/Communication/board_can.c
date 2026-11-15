#include "board_can.h"

DJI_Board DJI_Board_Target;
DJI_Board DJI_Board_FeedBack;


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

    /* 发送标准 CAN 数据帧 */
    return CAN_Transmit(hfdcan,
                        0U,
                        BOARD_ID_SPEED,
                        tx_data,
                        8U);
}

HAL_StatusTypeDef SmallBoard_Send_PID(FDCAN_HandleTypeDef *hfdcan, uint8_t pid_id, float kp, float ki, float kd)
{
    // 1. 变量定义放在最前面 (兼容C89)
    uint8_t tx_data[6] = {0};
    int32_t temp_kp, temp_ki, temp_kd;
    int16_t kp_val, ki_val, kd_val;
    
    // 防止超过 int16 最大量程 (32767 / 1000 = 32.767)，超了直接限幅，防止溢出变成负数导致电机暴走
    float limit = 32.767f; 
    if (kp > limit) kp = limit;   else if (kp < -limit) kp = -limit;
    if (ki > limit) ki = limit;   else if (ki < -limit) ki = -limit;
    if (kd > limit) kd = limit;   else if (kd < -limit) kd = -limit;

    // 2. 放大1000倍并转换
    temp_kp = (int32_t)(kp * 1000.0f);
    temp_ki = (int32_t)(ki * 1000.0f);
    temp_kd = (int32_t)(kd * 1000.0f);
    
    kp_val = (int16_t)temp_kp;
    ki_val = (int16_t)temp_ki;
    kd_val = (int16_t)temp_kd;

    // 3. 拆解高低字节 (强转 uint16_t 防止负数右移报错)
    tx_data[0] = (uint8_t)((uint16_t)kp_val & 0xFF);        // Kp 低字节
    tx_data[1] = (uint8_t)(((uint16_t)kp_val >> 8) & 0xFF); // Kp 高字节
    
    tx_data[2] = (uint8_t)((uint16_t)ki_val & 0xFF);        // Ki 低字节
    tx_data[3] = (uint8_t)(((uint16_t)ki_val >> 8) & 0xFF); // Ki 高字节
    
    tx_data[4] = (uint8_t)((uint16_t)kd_val & 0xFF);        // Kd 低字节
    tx_data[5] = (uint8_t)(((uint16_t)kd_val >> 8) & 0xFF); // Kd 高字节
    
    // 4. 调用底层 Can_Transmit 发送
    // header = 0 (标准帧), ID = 0x100 + pid_id, 长度 = 6
    return CAN_Transmit(hfdcan, 0U, 0x100U + pid_id, tx_data, 6U);
}

HAL_StatusTypeDef Send_Target_Pos(FDCAN_HandleTypeDef *hfdcan, float pos_mm[4])
{
    uint8_t tx_data[8];
    int16_t val;

    for (int i = 0; i < 4; i++) 
    {
        val = (int16_t)pos_mm[i]; // 转 int16
        tx_data[i*2]     = (uint8_t)(val & 0xFF);        // 低字节
        tx_data[i*2 + 1] = (uint8_t)((val >> 8) & 0xFF); // 高字节
    }
    
    // 调用你文件里的底层发送（header=0 是标准帧）
    return CAN_Transmit(hfdcan, 0U, 0x110U, tx_data, 8U);
}

HAL_StatusTypeDef Send_Target_Speed(FDCAN_HandleTypeDef *hfdcan, float speed_mps[4])
{
    uint8_t tx_data[8];
    int16_t val;
    
    for (int i = 0; i < 4; i++) 
    {
        val = (int16_t)(speed_mps[i] * 100.0f); 
        tx_data[i*2]     = (uint8_t)(val & 0xFF);
        tx_data[i*2 + 1] = (uint8_t)((val >> 8) & 0xFF);
    }

    return CAN_Transmit(hfdcan, 0U, 0x120U, tx_data, 8U);
}

static int16_t CAN_MergeInt16(uint8_t low_byte, uint8_t high_byte)
{
    uint16_t value = (uint16_t)low_byte | ((uint16_t)high_byte << 8);
    return (int16_t)value;
}

void CAN_Parse_SmallBoard(const can_frame_t *frame)
{
    if ((frame == NULL) || (frame->id_type != FDCAN_STANDARD_ID))
    {
        return;
    }

    if (((frame->id >= 0x180U) && (frame->id <= 0x182U) && (frame->length < 6U)) ||
        (((frame->id == 0x130U) || (frame->id == 0x140U)) && (frame->length < 8U)))
    {
        return;
    }

    switch (frame->id)
    {
        // ---------------- 第 0 套 PID ----------------
        case 0x180:
            DJI_Board_FeedBack.PID_K[0][0] = (float)CAN_MergeInt16(frame->data[0], frame->data[1]) / 1000.0f;
            DJI_Board_FeedBack.PID_K[0][1] = (float)CAN_MergeInt16(frame->data[2], frame->data[3]) / 1000.0f;
            DJI_Board_FeedBack.PID_K[0][2] = (float)CAN_MergeInt16(frame->data[4], frame->data[5]) / 1000.0f;
            break;

        // ---------------- 第 1 套 PID ----------------
        case 0x181:
            DJI_Board_FeedBack.PID_K[1][0] = (float)CAN_MergeInt16(frame->data[0], frame->data[1]) / 1000.0f;
            DJI_Board_FeedBack.PID_K[1][1] = (float)CAN_MergeInt16(frame->data[2], frame->data[3]) / 1000.0f;
            DJI_Board_FeedBack.PID_K[1][2] = (float)CAN_MergeInt16(frame->data[4], frame->data[5]) / 1000.0f;
            break;

        // ---------------- 第 2 套 PID ----------------
        case 0x182:
            DJI_Board_FeedBack.PID_K[2][0] = (float)CAN_MergeInt16(frame->data[0], frame->data[1]) / 1000.0f;
            DJI_Board_FeedBack.PID_K[2][1] = (float)CAN_MergeInt16(frame->data[2], frame->data[3]) / 1000.0f;
            DJI_Board_FeedBack.PID_K[2][2] = (float)CAN_MergeInt16(frame->data[4], frame->data[5]) / 1000.0f;
            break;

        // ---------------- 四条腿高度 ----------------
        case 0x130:
            for (int i = 0; i < 4; i++) {
                DJI_Board_FeedBack.leg_height_mm[i] =
                    (float)CAN_MergeInt16(frame->data[i * 2], frame->data[i * 2 + 1]);
            }
            break;

        // ---------------- 四个小轮速度 ----------------
        case 0x140:
            for (int i = 0; i < 4; i++) {
                DJI_Board_FeedBack.wheel_speed_mps[i] =
                    (float)CAN_MergeInt16(frame->data[i * 2], frame->data[i * 2 + 1]) / 100.0f;
            }
            break;

        default:
            break;
    }
}
