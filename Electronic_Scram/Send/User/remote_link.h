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

/* 功能：初始化模块并准备外设资源。 */
void RemoteLink_Init(void);
/* 功能：轮询输入或通信状态。 */
void RemoteLink_PollSwitches(void);
/* 功能：处理当前输入并更新模块状态。 */
void RemoteLink_Process(void);
/* 功能：转发通信数据。 */
void RemoteLink_ForwardRawFrame(void);

#ifdef __cplusplus
}
#endif
