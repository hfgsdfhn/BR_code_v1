#include "my_main.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h" 
#include "tim.h"    

uint16_t adc_dma_buf[4]; 
Remote_t remote;
volatile uint8_t tx_done_flag = 1; 
static uint32_t beep_start_time = 0;
static uint16_t beep_duration_ms = 0;
static uint8_t is_beeping = 0;

// ==========================================
// 纯普通按键列表 (共21个，已删除不存在的 PB0/PB1/PB2)
// ==========================================
Key_TypeDef keys[MY_KEY_MAX_NUM] = {
    {GPIOD, GPIO_PIN_10, 0, 0}, // 0
    {GPIOD, GPIO_PIN_11, 0, 0}, // 1
    {GPIOD, GPIO_PIN_12, 0, 0}, // 2
    {GPIOD, GPIO_PIN_13, 0, 0}, // 3
    {GPIOD, GPIO_PIN_14, 0, 0}, // 4
    {GPIOD, GPIO_PIN_15, 0, 0}, // 5
    
    {GPIOA, GPIO_PIN_8, 0, 0},  // 6
    {GPIOA, GPIO_PIN_11, 0, 0}, // 7
    {GPIOA, GPIO_PIN_12, 0, 0}, // 8
    {GPIOE, GPIO_PIN_6, 0, 0},  // 9
    {GPIOE, GPIO_PIN_3, 0, 0},  // 10
    {GPIOE, GPIO_PIN_4, 0, 0},  // 11
    {GPIOE, GPIO_PIN_5, 0, 0},  // 12
    {GPIOE, GPIO_PIN_2, 0, 0},  // 13
    {GPIOE, GPIO_PIN_1, 0, 0},  // 14
    {GPIOE, GPIO_PIN_0, 0, 0},  // 15
    
    {GPIOD, GPIO_PIN_3, 0, 0},  // 16
    {GPIOD, GPIO_PIN_4, 0, 0},  // 17
    {GPIOD, GPIO_PIN_5, 0, 0},  // 18
    {GPIOD, GPIO_PIN_6, 0, 0},  // 19
    {GPIOD, GPIO_PIN_7, 0, 0}   // 20
};

void Beep_Play(uint16_t duration_ms) {
    uint32_t now = HAL_GetTick();

    if (duration_ms > 0 && !is_beeping) {
        is_beeping = 1;
        beep_start_time = now;
        beep_duration_ms = duration_ms;
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }

    if (is_beeping && now - beep_start_time >= beep_duration_ms) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        is_beeping = 0;
    }
}

int16_t map_joy_adc(uint16_t val) {
    int16_t res = 0;
    // 假设 CENTER 是 2048, DEADZONE 是 30
    if (val > CENTER + DEADZONE) res = (val - CENTER) * 100 / (4095 - CENTER);
    else if (val < CENTER - DEADZONE) res = (val - CENTER) * 100 / CENTER;
    
    if (res > 100)  res = 100;
    if (res < -100) res = -100;
    return res;
}

// 判断肩键的 4 种状态 (加入了更宽的防抖死区)
uint8_t get_shoulder_state(uint16_t adc_val) {
    // ⚠️ 请务必根据你的实际电压分压调整以下阈值！
    if (adc_val < 200) return 0;        // 状态0：按下
    else if (adc_val < 1600) return 1;  // 状态1：左拨
    else if (adc_val < 2900) return 2;  // 状态2：不按 (中位)
    else return 3;                      // 状态3：右拨
}

void scan_joystick(void) {
    static uint8_t last_shoulder_left = 0;
    static uint8_t last_shoulder_right = 0;
    static uint8_t shoulder_state_initialized = 0;

    // 摇杆数据映射 (-100 到 100)
    remote.joy_x = -map_joy_adc(adc_dma_buf[0]); // PC4
    remote.joy_y = -map_joy_adc(adc_dma_buf[1]); // PC5
    
    // PA7 和 PC1 读取 4 种状态
    remote.shoulder_left = get_shoulder_state(adc_dma_buf[2]);
    remote.shoulder_right = get_shoulder_state(adc_dma_buf[3]);

    // 状态改变时触发蜂鸣器
    if (shoulder_state_initialized &&
        (remote.shoulder_left != last_shoulder_left ||
         remote.shoulder_right != last_shoulder_right)) {
        Beep_Play(30);
    }

    last_shoulder_left = remote.shoulder_left;
    last_shoulder_right = remote.shoulder_right;
    shoulder_state_initialized = 1;
}

// 扫描 6 个物理拨杆 (不触发蜂鸣器)
void scan_switches(void) {
    uint8_t sw_state = 0;
    
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) sw_state |= (1 << 0);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) sw_state |= (1 << 1);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) sw_state |= (1 << 2);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) sw_state |= (1 << 3);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) sw_state |= (1 << 4);
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) sw_state |= (1 << 5);

    remote.byte1 = sw_state; // 拨杆状态存 byte1
}

// 扫描 21 个普通按键 (优化了消抖算法)
void scan_keys(void) {
    uint8_t b1 = 0, b2 = 0, b3 = 0;
    const uint8_t DEBOUNCE_THRESHOLD = 3; // 3次*5ms = 15ms消抖
    
    for (uint8_t i = 0; i < MY_KEY_MAX_NUM; i++) {
        uint8_t level = HAL_GPIO_ReadPin(keys[i].GPIO_Port, keys[i].GPIO_Pin);
        
        if (level == GPIO_PIN_RESET) {
            // 检测到低电平（按下）
            if (keys[i].count < DEBOUNCE_THRESHOLD) {
                keys[i].count++;
                if (keys[i].count == DEBOUNCE_THRESHOLD) {
                    keys[i].state = 1; // 连续3次低电平，确认按下
                }
            }
        } else {
            // 检测到高电平（松开），直接递减，加速响应
            if (keys[i].count > 0) {
                keys[i].count--;
                if (keys[i].count == 0) {
                    keys[i].state = 0; // 确认松开
                }
            }
        }
        
        if (keys[i].state == 1) {
            if (i < 8)        b1 |= (1 << i);          // 按键 0-7
            else if (i < 16)  b2 |= (1 << (i - 8));    // 按键 8-15
            else if (i < 21)  b3 |= (1 << (i - 16));   // 按键 16-20 (共5个)
        }
    }
    
    remote.byte2 = b1; 
    remote.byte3 = b2; 
    remote.byte4 = b3; 

    // 蜂鸣器触发逻辑
    static uint8_t last_b1 = 0, last_b2 = 0, last_b3 = 0;
    if (((b1 != last_b1) && b1) || 
        ((b2 != last_b2) && b2) || 
        ((b3 != last_b3) && b3)) {
        Beep_Play(30); 
    }
    last_b1 = b1; 
    last_b2 = b2; 
    last_b3 = b3;
}

uint8_t Calculate_CRC8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

void Send_Remote_Data(void) {
    remote.tx_frame[0] = FRAME_HEAD1;
    remote.tx_frame[1] = FRAME_HEAD2;
    
    // 摇杆 X 轴 (2字节)
    remote.tx_frame[2] = (uint8_t)(remote.joy_x & 0xFF);        
    remote.tx_frame[3] = (uint8_t)((remote.joy_x >> 8) & 0xFF); 
    
    // 摇杆 Y 轴 (2字节)
    remote.tx_frame[4] = (uint8_t)(remote.joy_y & 0xFF);        
    remote.tx_frame[5] = (uint8_t)((remote.joy_y >> 8) & 0xFF); 

    // 肩键状态 (2字节)
    remote.tx_frame[6] = remote.shoulder_left;
    remote.tx_frame[7] = remote.shoulder_right;

    // 拨杆 (1字节)
    remote.tx_frame[8] = remote.byte1; 
    
    // 纯按键 (3字节)
    remote.tx_frame[9]  = remote.byte2; 
    remote.tx_frame[10] = remote.byte3; 
    remote.tx_frame[11] = remote.byte4; 

    // CRC 校验 (索引 2~11，共 10 个字节)
    remote.tx_frame[12] = Calculate_CRC8(&remote.tx_frame[2], 10);
    
    remote.tx_frame[13] = FRAME_TAIL1;
    remote.tx_frame[14] = FRAME_TAIL2;

    if (tx_done_flag == 1 && huart6.gState == HAL_UART_STATE_READY) {
        tx_done_flag = 0;
        HAL_UART_Transmit_IT(&huart6, remote.tx_frame, 15); // 总长 15 字节
    }
}

void My_Init(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, 4);
}

void My_Loop(void) {
    static uint32_t last_tx_time = 0;
    static uint32_t last_key_time = 0;
    uint32_t current_time = HAL_GetTick(); 

    if (current_time - last_key_time >= 5) {
        last_key_time = current_time;
        scan_switches(); 
        scan_keys();
    }

    if (current_time - last_tx_time >= 20) {
        last_tx_time = current_time;
        scan_joystick();
        Send_Remote_Data(); 
    }
    
    Beep_Play(0); 
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        tx_done_flag = 1;
    }
}