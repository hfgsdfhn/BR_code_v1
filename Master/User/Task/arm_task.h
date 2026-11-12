/*
 * @file arm_task.h
 * @brief arm_task模块的用户接口与实现。
 */

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

/* 代码说明：相关模块实现。 */
extern volatile uint8_t arm_joint_zero_calibration[2][5];

/* 功能：执行 Arm_TaskInit 函数对应的模块操作。 */
void Arm_TaskInit(void);
/* 功能：执行 Arm_TaskStop 函数对应的模块操作。 */
void Arm_TaskStop(void);
/* 功能：执行 Arm_Task 函数对应的模块操作。 */
void Arm_Task(void *argument);
/* 功能：执行 Arm_TxTask 函数对应的模块操作。 */
void Arm_TxTask(void *argument);
/* 功能：执行 Arm_OnFrame 函数对应的模块操作。 */
void Arm_OnFrame(const can_frame_t *frame);
/* 功能：执行 Arm_TxQueueInit 函数对应的模块操作。 */
bool Arm_TxQueueInit(void);

#endif
