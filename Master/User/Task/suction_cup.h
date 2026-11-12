#ifndef SUCTION_CUP_H
#define SUCTION_CUP_H
#include <stdint.h>
#include "STP-23L.h"
#include "../Driver/Motor/DJ_M2006/m2006.h"
#include "../Driver/Motor/DJ_M3508/m3508.h"
#include "../Peripheral/CAN/bsp_can.h"

typedef struct
{
    uint8_t frame[STP23L_FRAME_SIZE];
    uint16_t index;
} suction_stp23l_parser_t;

extern m2006_t m2006_ucup[2];
extern m2006_t m2006_dcup[2];
extern m3508_t m3508_dcup[1];

extern volatile uint8_t suction_enable;
extern volatile float suction_target[5];
/* Measured angles in degrees, updated on received feedback. */
extern volatile float suction_feedback_angle[5];
extern volatile float g_up_stp23l_distance_cm;
extern volatile float g_down_stp23l_distance_cm;

void SuctionCup_Init(void);
void SuctionCup_OnFrame(const can_frame_t *frame);
void SuctionCup_Service(void);
void SuctionCup_Task(void *argument);
#endif
