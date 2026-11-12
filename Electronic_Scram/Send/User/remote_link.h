/*
 * @file remote_link.h
 * @brief remote_link模块的用户接口与实现。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* 功能：执行 RemoteLink_Init 函数对应的模块操作。 */
void RemoteLink_Init(void);
/* 功能：执行 RemoteLink_PollSwitches 函数对应的模块操作。 */
void RemoteLink_PollSwitches(void);
/* 功能：执行 RemoteLink_Process 函数对应的模块操作。 */
void RemoteLink_Process(void);
/* 功能：执行 RemoteLink_ForwardRawFrame 函数对应的模块操作。 */
void RemoteLink_ForwardRawFrame(void);

#ifdef __cplusplus
}
#endif
