/*
 * @file remote.c
 * @brief 遥控器串口接收协议解析
 */

#include "remote.h"
<<<<<<< HEAD
#include "stm32h7xx_hal.h"
=======
/* E34-2G4H27D：M0=0、M1=0 为透明传输；模块参数由上位机配置，固件不发送配置命令。 */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
#include <string.h>

remote_t remote = {0};

/* 内部接收缓冲区，用于处理半包/粘包 */
#define REMOTE_FRAME_LEN 15U
static uint8_t rx_buffer[64] = {0};
static uint16_t rx_index = 0;

<<<<<<< HEAD
/* CRC-8 计算 (多项式 0x07, 初始值 0x00) */
static uint8_t Remote_CalcCrc8(const uint8_t *data, uint16_t len)
=======
/* 功能：执行 remote_parse_byte 模块操作并维护相关状态。 */
static void remote_parse_byte(uint8_t byte)
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
{
    uint8_t crc = 0x00U;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* 解析单帧数据 */
static uint8_t Remote_ParseFrame(const uint8_t *frame, uint32_t current_tick)
{
    /* 1. 检查帧尾 */
    if (frame[13] != 0x0DU || frame[14] != 0x0AU)
    {
<<<<<<< HEAD
=======
        if (byte == REMOTE_FRAME_HEADER_1)
            remote_parser[remote_parser_len++] = byte;
        else
            remote_parser_len = (byte == REMOTE_FRAME_HEADER_0) ? 1U : 0U;
        return;
    }
    remote_parser[remote_parser_len++] = byte;
    if (remote_parser_len == REMOTE_FRAME_SIZE)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        remote_state.joystick = remote_parser[2];
        remote_state.group1 = remote_parser[3];
        remote_state.shoulder = remote_parser[4];
        remote_state.group2 = remote_parser[5];
        remote_state.group3 = remote_parser[6];
        remote_state.group4 = remote_parser[7];
        remote_state.received_tick = HAL_GetTick();
        remote_state.valid = 1U;
        remote_state.valid_frames = ++remote_valid_count;
        __set_PRIMASK(primask);
        remote_parser_len = 0U;
    }
}


/* 功能：初始化模块并准备外设资源。 */
void Remote_Init(void)
{
    memset(&remote_state, 0, sizeof(remote_state));
    remote_parser_len = 0U;
    remote_valid_count = 0U;
}


/* 功能：执行 Remote_RxBytes 模块操作并维护相关状态。 */
void Remote_RxBytes(const uint8_t *data, uint16_t size)
{
    while (data != NULL && size-- != 0U)
        remote_parse_byte(*data++);
}


/* 功能：记录并处理错误状态。 */
void Remote_UartError(void)
{

    remote_parser_len = 0U;
    remote_state.valid = 0U;
}


/* 功能：执行 Remote_GetState 模块操作并维护相关状态。 */
uint8_t Remote_GetState(remote_state_t *state)
{
    uint32_t primask;
    if (state == NULL)
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
        return 0U;
    }

    /* 2. 检查 CRC (覆盖偏移 2~11, 共 10 字节) */
    if (Remote_CalcCrc8(&frame[2], 10) != frame[12])
    {
        return 0U;
    }

    /* 3. 提取数据并加锁保护 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    /* X摇杆：小端有符号16位 */
    remote.joy_x = (int16_t)((uint16_t)frame[2] | ((uint16_t)frame[3] << 8));
    /* Y摇杆：小端有符号16位 */
    remote.joy_y = (int16_t)((uint16_t)frame[4] | ((uint16_t)frame[5] << 8));

    remote.shoulder_left  = frame[6];
    remote.shoulder_right = frame[7];
    remote.switch_state   = frame[8];

    remote.key[0] = frame[9];
    remote.key[1] = frame[10];
    remote.key[2] = frame[11];

    /* 4. 更新状态 */
    remote.last_update_tick = current_tick;
    remote.online = 1U;
    __set_PRIMASK(primask);
    return 1U;
}

<<<<<<< HEAD
static uint8_t Remote_ProcessBuffer(uint32_t current_tick)
=======

/* 功能：执行 Remote_IsOnline 模块操作并维护相关状态。 */
uint8_t Remote_IsOnline(uint32_t timeout_ms)
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
{
    uint8_t parsed_frames = 0U;
    uint16_t i = 0U;

    while (i + REMOTE_FRAME_LEN <= rx_index)
    {
        if (rx_buffer[i] == 0xAAU && rx_buffer[i + 1U] == 0x55U)
        {
            /* 只有在解析成功时才跳过整帧长度 */
            if (Remote_ParseFrame(&rx_buffer[i], current_tick))
            {
                parsed_frames++;
                i += REMOTE_FRAME_LEN;
            }
            else
            {
                /* 如果 CRC 或帧尾错误，只往前移 1 字节，重新寻找下一个合法帧头 */
                i++;
            }
        }
        else
        {
            /* 没找到帧头，继续往后找 */
            i++;
        }
    }

    /* 清理已处理的数据 */
    if (i > 0U)
    {
        uint16_t remain = rx_index - i;
        if (remain > 0U)
        {
            memmove(rx_buffer, &rx_buffer[i], remain);
        }
        rx_index = remain;
    }

    return parsed_frames;
}

<<<<<<< HEAD
void Remote_Init(void)
=======

/* 功能：执行 Remote_GetValidFrameCount 模块操作并维护相关状态。 */
uint32_t Remote_GetValidFrameCount(void)
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(&remote, 0, sizeof(remote));
    rx_index = 0U;
    __set_PRIMASK(primask);
}

/* 接收函数：将串口数据逐字节压入内部缓冲，寻找合法帧 */
uint8_t Remote_Receive(const uint8_t *data, uint16_t len)
 {
    uint8_t parsed_frames = 0U;
    uint32_t current_tick = HAL_GetTick();

    if (data == NULL || len == 0U)
    {
        return 0U;
    }

    while (len > 0U)
    {
        uint16_t available = (uint16_t)(sizeof(rx_buffer) - rx_index);
        uint16_t copy_size = (len < available) ? len : available;

        memcpy(&rx_buffer[rx_index], data, copy_size);
        rx_index += copy_size;
        data += copy_size;
        len = (uint16_t)(len - copy_size);
        
        parsed_frames += Remote_ProcessBuffer(current_tick);
    }

    return parsed_frames;
}

void Remote_UartError(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_index = 0U;
    remote.online = 0U;
    __set_PRIMASK(primask);
}

uint8_t Remote_GetState(remote_t *state)
{
    uint32_t primask;

    if (state == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *state = remote;
    __set_PRIMASK(primask);
    return state->online;
}