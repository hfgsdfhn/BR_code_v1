#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void RemoteLink_Init(void);
void RemoteLink_PollSwitches(void);
void RemoteLink_Process(void);
void RemoteLink_ForwardRawFrame(void);

#ifdef __cplusplus
}
#endif
