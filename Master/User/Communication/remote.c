/*
 * @file remote.c
 * @brief remote模块的用户接口与实现。
 */

#include "remote.h"
#include <string.h>

static uint8_t remote_parser[REMOTE_FRAME_SIZE];
static uint8_t remote_parser_len;
static remote_state_t remote_state;
static uint32_t remote_valid_count;


/* 功能：执行 remote_parse_byte 函数对应的模块操作。 */
static void remote_parse_byte(uint8_t byte)
{
    if (remote_parser_len == 0U)
    {
        if (byte == REMOTE_FRAME_HEADER_0)
            remote_parser[remote_parser_len++] = byte;
        return;
    }
    if (remote_parser_len == 1U)
    {
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


/* 功能：执行 Remote_Init 函数对应的模块操作。 */
void Remote_Init(void)
{
    memset(&remote_state, 0, sizeof(remote_state));
    remote_parser_len = 0U;
    remote_valid_count = 0U;
}


/* 功能：执行 Remote_RxBytes 函数对应的模块操作。 */
void Remote_RxBytes(const uint8_t *data, uint16_t size)
{
    while (data != NULL && size-- != 0U)
        remote_parse_byte(*data++);
}


/* 功能：执行 Remote_UartError 函数对应的模块操作。 */
void Remote_UartError(void)
{

    remote_parser_len = 0U;
    remote_state.valid = 0U;
}


/* 功能：执行 Remote_GetState 函数对应的模块操作。 */
uint8_t Remote_GetState(remote_state_t *state)
{
    uint32_t primask;
    if (state == NULL)
        return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    if (remote_state.valid == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    *state = remote_state;
    __set_PRIMASK(primask);
    return 1U;
}


/* 功能：执行 Remote_IsOnline 函数对应的模块操作。 */
uint8_t Remote_IsOnline(uint32_t timeout_ms)
{
    return (uint8_t)(remote_state.valid != 0U &&
                     (HAL_GetTick() - remote_state.received_tick) <= timeout_ms);
}


/* 功能：执行 Remote_GetValidFrameCount 函数对应的模块操作。 */
uint32_t Remote_GetValidFrameCount(void)
{
    return remote_valid_count;
}
