#include "board_can.h"

volatile float PID_K[3][3] = {
    {1.0f, 0.0f, 0.0f},  // 第 0 套：腿部位置环
    {4.0f, 0.0f,0.0f},  // 第 1 套：腿部速度环
    {15.0f, 0.5f, 10.0f}   // 第 2 套：轮子速度环
};
volatile float leg_height_mm[4] = {0};
volatile float wheel_speed_mps[4] = {0};

#define MERGE_INT16(low, high) ((int16_t)(((high) << 8) | (low)))

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length);

void CAN_Parse_SmallBoard(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) 
{
    // 直接通过指针取 ID
    switch (rx_header->StdId) 
    {
        // ---------------- 第 0 套 PID ----------------
        case 0x100: 
            PID_K[0][0] = (float)MERGE_INT16(rx_data[0], rx_data[1]) / 1000.0f;
            PID_K[0][1] = (float)MERGE_INT16(rx_data[2], rx_data[3]) / 1000.0f;
            PID_K[0][2] = (float)MERGE_INT16(rx_data[4], rx_data[5]) / 1000.0f;
            break;

        // ---------------- 第 1 套 PID ----------------
        case 0x101: 
            PID_K[1][0] = (float)MERGE_INT16(rx_data[0], rx_data[1]) / 1000.0f;
            PID_K[1][1] = (float)MERGE_INT16(rx_data[2], rx_data[3]) / 1000.0f;
            PID_K[1][2] = (float)MERGE_INT16(rx_data[4], rx_data[5]) / 1000.0f;
            break;

        // ---------------- 第 2 套 PID ----------------
        case 0x102: 
            PID_K[2][0] = (float)MERGE_INT16(rx_data[0], rx_data[1]) / 1000.0f;
            PID_K[2][1] = (float)MERGE_INT16(rx_data[2], rx_data[3]) / 1000.0f;
            PID_K[2][2] = (float)MERGE_INT16(rx_data[4], rx_data[5]) / 1000.0f;
            break;

        // ---------------- 四条腿高度 ----------------
        case 0x110: 
            for (int i = 0; i < 4; i++) {
                leg_height_mm[i] = (float)MERGE_INT16(rx_data[i*2], rx_data[i*2+1]);
                if(leg_height_mm[i] < 0.0f) leg_height_mm[i] = 0.0f;
                if(leg_height_mm[i] > 800.0f) leg_height_mm[i] = 800.0f;
            }
            break;

        // ---------------- 四个小轮速度 ----------------
        case 0x120:
            for (int i = 0; i < 4; i++) {
                wheel_speed_mps[i] = (float)MERGE_INT16(rx_data[i*2], rx_data[i*2+1]) / 100.0f;
            }
            break;

        default:
            break;
    }
}

static HAL_StatusTypeDef Can_Transmit(CAN_HandleTypeDef *hcan, uint8_t header,
                                      uint32_t id, const uint8_t *data,
                                      uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox;
    uint32_t start_tick;

    if ((hcan == NULL) || (data == NULL) || (length > 8U) || (header > 1U))
    {
        return HAL_ERROR;
    }

    if (header == 0U)
    {
        if (id > 0x7FFU)
        {
            return HAL_ERROR;
        }
        tx_header.StdId = id;
        tx_header.IDE = CAN_ID_STD;
    }
    else
    {
        if (id > 0x1FFFFFFFU)
        {
            return HAL_ERROR;
        }
        tx_header.ExtId = id;
        tx_header.IDE = CAN_ID_EXT;
    }

    /* 最多等待2ms，直到发送邮箱空闲 */
    start_tick = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= 2U)
        {
            return HAL_TIMEOUT;
        }
    }

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &mailbox);
}

void SmallBoard_Send_PID(CAN_HandleTypeDef *hcan, uint8_t pid_id, float kp, float ki, float kd)
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
    Can_Transmit(hcan, 0U, 0x180U + pid_id, tx_data, 6);
}

void Send_Feedback_Pos(CAN_HandleTypeDef *hcan, float pos_mm[4])
{
    uint8_t tx_data[8];
    int16_t val;

    for (int i = 0; i < 4; i++) 
    {
        val = (int16_t)pos_mm[i] ; // 转 int16
        tx_data[i*2]     = (uint8_t)(val & 0xFF);        // 低字节
        tx_data[i*2 + 1] = (uint8_t)((val >> 8) & 0xFF); // 高字节
    }
    
    // 调用你文件里的底层发送（header=0 是标准帧）
    Can_Transmit(hcan, 0U, 0x130, tx_data, 8);
}

/**
 * @brief 发送帧5：四个小轮实际速度 (ID: 0x140)
 * @param speed_mps 传入四个电机的实际速度数组 (单位: m/s)
 */
void Send_Feedback_Speed(CAN_HandleTypeDef *hcan, float speed_mps[4])
{
    uint8_t tx_data[8];
    int16_t val;
    
    for (int i = 0; i < 4; i++) 
    {
        // 单位是 m/s 的 100 倍，所以乘以 100
        val = (int16_t)(speed_mps[i] * 100.0f); 
        tx_data[i*2]     = (uint8_t)(val & 0xFF);
        tx_data[i*2 + 1] = (uint8_t)((val >> 8) & 0xFF);
    }
    
    Can_Transmit(hcan, 0U, 0x140, tx_data, 8);
}
