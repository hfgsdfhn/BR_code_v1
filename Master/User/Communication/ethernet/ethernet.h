
#ifndef __ETHERNET_H
#define __ETHERNET_H

#include "status_task.h"

/* W5500 初始化和 UDP 任务 */
void W5500_Port_Init(void);
void W5500_UdpInit(void);

osStatus_t W5500_SendStatus(const send_to_ethernet_t *status, const uint8_t ip[4], uint16_t port);

#endif
