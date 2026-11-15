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

extern volatile uint8_t arm_joint_zero_calibration[2][5];

/* 功能：初始化Arm_TaskInit对应的外设、参数和运行状态。 */
void Arm_TaskInit(void);
/* 功能：停止Arm_TaskStop对应的输出并清理运行状态。 */
void Arm_TaskStop(void);
/* 功能：执行Arm_Task对应的周期任务并维护运行状态。 */
void Arm_Task(void *argument);
/* 功能：执行Arm_TxTask对应的周期任务并维护运行状态。 */
void Arm_TxTask(void *argument);
/* 功能：接收并解析Arm_OnFrame对应的输入数据，更新模块状态。 */
void Arm_OnFrame(const can_frame_t *frame);
/* 功能：将Arm_TxQueueInit对应的目标数据加入执行队列。 */
bool Arm_TxQueueInit(void);

#endif
