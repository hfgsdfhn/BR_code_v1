#ifndef HAND_REMOTE_H
#define HAND_REMOTE_H

#include <stdint.h>
#include "main.h"

#define REMOTE_FRAME_HEADER_0 0xA5U
#define REMOTE_FRAME_HEADER_1 0x5AU
#define REMOTE_FRAME_SIZE 8U
#define REMOTE_TX_PERIOD_MS 20U
void Remote_Init(void);
void Remote_Loop(void);

#endif
