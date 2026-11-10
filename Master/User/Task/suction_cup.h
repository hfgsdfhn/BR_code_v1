#ifndef SUCTION_CUP_H
#define SUCTION_CUP_H
#include <stdint.h>
extern volatile uint8_t g_suction_enable;
extern volatile float g_suction_target_deg[4];
void SuctionCup_Task(void *argument);
#endif
