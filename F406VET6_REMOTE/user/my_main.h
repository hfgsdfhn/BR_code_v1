#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "main.h"
#include "stdint.h"

#define DEADZONE 50
#define CENTER 2048
// 修改为实际的 21 个按键
#define MY_KEY_MAX_NUM 21 

#define FRAME_HEAD1 0xAA
#define FRAME_HEAD2 0x55
#define FRAME_TAIL1 0x0D
#define FRAME_TAIL2 0x0A

typedef struct
{
    int16_t joy_x;   // PC4 摇杆左右
    int16_t joy_y;   // PC5 摇杆上下
    uint8_t shoulder_left;  // 左肩键状态 (0~3)
    uint8_t shoulder_right; // 右肩键状态 (0~3)
    uint8_t byte1;   // 6个物理拨杆
    uint8_t byte2;   // 按键 0~7
    uint8_t byte3;   // 按键 8~15
    uint8_t byte4;   // 按键 16~20
    
    uint8_t tx_frame[20]; //扩容到 20 字节，实际发送 15 字节
} Remote_t;

typedef struct {
    GPIO_TypeDef* GPIO_Port; 
    uint16_t GPIO_Pin;       
    uint8_t  count;          
    uint8_t  state;          
} Key_TypeDef;

void My_Init(void);
void My_Loop(void);

#endif