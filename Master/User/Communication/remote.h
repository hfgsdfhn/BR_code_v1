#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define REMOTE_FRAME_HEADER_0 0xA5U
#define REMOTE_FRAME_HEADER_1 0x5AU
#define REMOTE_FRAME_SIZE 8U

typedef struct
{
    uint8_t joystick; /* byte 2: low nibble=X, high nibble=Y */
    uint8_t group1;   /* byte 3 */
    uint8_t shoulder; /* byte 4: bit6/bit7 */
    uint8_t group2;   /* byte 5 */
    uint8_t group3;   /* byte 6 */
    uint8_t group4;   /* byte 7 */
    uint32_t received_tick;
    uint32_t valid_frames;
    uint8_t valid;
} remote_state_t;

void Remote_Init(void);
void Remote_RxByte(void);
void Remote_RxBytes(const uint8_t *data, uint16_t size);
void Remote_UartError(void);
uint8_t Remote_GetState(remote_state_t *state);
uint8_t Remote_IsOnline(uint32_t timeout_ms);
uint32_t Remote_GetValidFrameCount(void);

#endif
