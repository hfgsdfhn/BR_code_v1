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

/* 模块实现细节。 */
extern volatile uint8_t arm_joint_zero_calibration[2][5];

/* 功能：初始化模块并准备外设资源。 */
void Arm_TaskInit(void);
/* 功能：执行周期任务主流程。 */
void Arm_TaskStop(void);
/* 功能：执行周期任务主流程。 */
void Arm_Task(void *argument);
/* 功能：执行周期任务主流程。 */
void Arm_TxTask(void *argument);
/* 功能：接收并解析一帧通信数据。 */
void Arm_OnFrame(const can_frame_t *frame);
/* 功能：初始化模块并准备外设资源。 */
bool Arm_TxQueueInit(void);

#endif
