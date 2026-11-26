
#ifndef SUCTION_CUP_H
#define SUCTION_CUP_H
#include "../Driver/Motor/DJ_M2006/m2006.h"
#include "../Driver/Motor/DJ_M3508/m3508.h"
#include "../Peripheral/CAN/bsp_can.h"
#include "STP-23L.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t frame[STP23L_FRAME_SIZE];
    uint16_t index;
} suction_stp23l_parser_t;

extern m2006_t m2006_ucup[2];
extern m2006_t m2006_dcup[2];
extern m3508_t m3508_dcup[1];

extern volatile uint8_t suction_enable;
extern volatile float suction_target[2][3];

extern volatile float suction_feedback_angle[2][3];
extern volatile float ustp23l_distance_cm;
extern volatile float dstp23l_distance_cm;

void UStp23lRx(const uint8_t *data, uint16_t size);
void DStp23lRx(const uint8_t *data, uint16_t size);

void SuctionCup_Init(void);
void SuctionCup_OnFrame(const can_frame_t *frame);
void SuctionCup_Service(void);
void SuctionCup_Task(void *argument);
#endif
