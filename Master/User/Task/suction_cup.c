/* Optional adapter: four CAN2 auxiliaries. Not scheduled by the BR default profile. */
#include "suction_cup.h"
#include "cmsis_os2.h"
#include "upper.h"
volatile uint8_t g_suction_enable;
volatile float g_suction_target_deg[4];
void SuctionCup_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        unsigned i;
        for (i = 0; i < 4; i++)
            if (i != BR_RIGHT_ROOT_M2006_INDEX)
                (void)Upper_SetM2006(2U, (uint8_t)i, g_suction_enable ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                     g_suction_target_deg[i]);
        osDelay((osKernelGetTickFreq() * 10U + 999U) / 1000U);
    }
}
