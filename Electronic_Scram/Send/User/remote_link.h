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

/* 功能：初始化RemoteLink_Init对应的外设、参数和运行状态。 */
void RemoteLink_Init(void);
/* 功能：轮询并更新RemoteLink_PollSwitches对应的输入和状态。 */
void RemoteLink_PollSwitches(void);
/* 功能：轮询并更新RemoteLink_Process对应的输入和状态。 */
void RemoteLink_Process(void);
/* 功能：发送RemoteLink_ForwardRawFrame对应的数据或控制指令。 */
void RemoteLink_ForwardRawFrame(void);

#ifdef __cplusplus
}
#endif
