/*
 * @file remote.h
 * @brief 遥控器串口接收协议解析
 */

#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>

/* 遥控器数据结构体 */
typedef struct
{
    int16_t  joy_x;          /* X摇杆映射值 (偏移2,3, 小端有符号, -100~100) */
    int16_t  joy_y;          /* Y摇杆映射值 (偏移4,5, 小端有符号, -100~100) */
    uint8_t  shoulder_left;  /* 左肩键状态 0~3 (偏移6) */
    uint8_t  shoulder_right; /* 右肩键状态 0~3 (偏移7) */
    uint8_t  switch_state;   /* 拨杆状态 (偏移8) */
    uint8_t  key[3];         /* 普通按键 K0~K20 (偏移9,10,11) 对应 byte2,3,4 */
    uint32_t last_update_tick; /* 最后一次成功解析的时间戳，用于断线检测 */
    uint8_t  online;         /* 在线标志 1:在线 0:掉线 */
} remote_t;

extern remote_t remote;

/* 接收函数：将串口收到的数据喂给此函数进行解析
 * 返回成功解析出的完整帧数 */
uint8_t Remote_Receive(const uint8_t *data, uint16_t len);
void Remote_Init(void);
void Remote_UartError(void);
uint8_t Remote_GetState(remote_t *state);

#endif /* REMOTE_H */