/*
 * @file remote.h
 * @brief 遥控器串口接收协议解析
 */

#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>
/* ---- remote.key[0] : K0~K7 ---- */
#define KEY_PD10   ((remote.key[0] >> 0) & 0x01U)
#define KEY_PD11   ((remote.key[0] >> 1) & 0x01U)
#define KEY_PD12   ((remote.key[0] >> 2) & 0x01U)
#define KEY_PD13   ((remote.key[0] >> 3) & 0x01U)
#define KEY_PD14   ((remote.key[0] >> 4) & 0x01U)
#define KEY_PD15   ((remote.key[0] >> 5) & 0x01U)
#define KEY_PA8    ((remote.key[0] >> 6) & 0x01U)
#define KEY_PA11   ((remote.key[0] >> 7) & 0x01U)

/* ---- remote.key[1] : K8~K15 ---- */
#define KEY_PA12   ((remote.key[1] >> 0) & 0x01U)
#define KEY_PE6    ((remote.key[1] >> 1) & 0x01U)
#define KEY_PE3    ((remote.key[1] >> 2) & 0x01U)
#define KEY_PE4    ((remote.key[1] >> 3) & 0x01U)
#define KEY_PE5    ((remote.key[1] >> 4) & 0x01U)
#define KEY_PE2    ((remote.key[1] >> 5) & 0x01U)
#define KEY_PE1    ((remote.key[1] >> 6) & 0x01U)
#define KEY_PE0    ((remote.key[1] >> 7) & 0x01U)

/* ---- remote.key[2] : K16~K20 ---- */
#define KEY_PD3    ((remote.key[2] >> 0) & 0x01U)
#define KEY_PD4    ((remote.key[2] >> 1) & 0x01U)
#define KEY_PD5    ((remote.key[2] >> 2) & 0x01U)
#define KEY_PD6    ((remote.key[2] >> 3) & 0x01U)
#define KEY_PD7    ((remote.key[2] >> 4) & 0x01U)

typedef enum
{
    REMOTE_KEY_PE0 = 0U,
    REMOTE_KEY_PE1,
    REMOTE_KEY_PE2,
    REMOTE_KEY_PE3,
    REMOTE_KEY_PE4,
    REMOTE_KEY_PE5,
    REMOTE_KEY_PE6,
    REMOTE_KEY_SHOULDER_RIGHT_0,
    REMOTE_KEY_SHOULDER_RIGHT_1,
    REMOTE_KEY_SHOULDER_RIGHT_2,
    REMOTE_KEY_SHOULDER_RIGHT_3,
    REMOTE_KEY_COUNT
} remote_key_id_t;

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
void Remote_RxBytes(const uint8_t *data, uint16_t len);
void Remote_Init(void);
void Remote_UartError(void);
uint8_t key_is_pressed(remote_key_id_t key_id);
uint8_t Remote_GetState(remote_t *state);

#endif /* REMOTE_H */
