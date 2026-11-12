#ifndef ARM_TASK_H
#define ARM_TASK_H
#include "../Peripheral/CAN/bsp_can.h"

#include <stdbool.h>
#include <stdint.h>
extern volatile uint8_t arm_gpio_pb6;
extern volatile uint8_t arm_gpio_pb7;
extern volatile uint8_t arm_gpio_pd7;
extern volatile uint8_t arm_gpio_pd6;
extern volatile uint8_t arm_gpio_pd1;
extern volatile uint8_t arm_gpio_pd3;
extern volatile uint8_t arm_gpio_pd4;
extern volatile uint8_t arm_gpio_pd5;

/* 0 = idle; a 0->1 edge requests zero calibration for the joint. */
extern volatile uint8_t arm_joint_zero_calibration[2][5];

void Arm_TaskInit(void);
void Arm_TaskStop(void);
void Arm_Task(void *argument);
void Arm_TxTask(void *argument);
void Arm_OnFrame(const can_frame_t *frame);
bool Arm_TxQueueInit(void);

#endif
