
#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>

typedef struct
{
    int16_t  joy_x;
    int16_t  joy_y;
    uint8_t  shoulder_left;
    uint8_t  shoulder_right;
    uint8_t  switch_state;
    uint8_t  key[3];
    uint32_t last_update_tick;
    uint8_t  online;
} remote_t;

extern remote_t remote;

uint8_t Remote_Receive(const uint8_t *data, uint16_t len);
void Remote_RxBytes(const uint8_t *data, uint16_t len);
void Remote_Init(void);
void Remote_UartError(void);
uint8_t Remote_GetState(remote_t *state);

#endif
