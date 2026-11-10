#include "arm_task.h"
#include "can_board.h"
#include "cmsis_os2.h"
#include "my_main.h"
#include "upper.h"

void Arm_Task(void *argument)
{
    uint32_t next = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        upper_state.enable = (g_motor_control_enable == 1U);
        Upper_QueueArmPositions(upper_state.enable != 0U);
        next += (osKernelGetTickFreq() * 2U + 999U) / 1000U;
        if (osDelayUntil(next) != osOK)
        {
            next = osKernelGetTickCount();
        }
    }
}
