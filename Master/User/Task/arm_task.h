
#ifndef ARM_TASK_H
#define ARM_TASK_H
#include "../Peripheral/CAN/bsp_can.h"
#include "../Mechanism/Upper/upper.h"

#include <stdbool.h>
#include <stdint.h>
extern volatile uint8_t gpio_pb6;
extern volatile uint8_t gpio_pb7;
extern volatile uint8_t gpio_pd7;
extern volatile uint8_t gpio_pd6;
extern volatile uint8_t gpio_pd1;
extern volatile uint8_t gpio_pd3;
extern volatile uint8_t gpio_pd4;
extern volatile uint8_t gpio_pd5;
extern volatile uint8_t arm_low_low_mode_enable;

extern volatile float arm_pose_pd11[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13_1[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13_2[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd14[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15_2[2][ARM_JOINT_COUNT];
extern volatile float xarm_pose_pd13[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd11[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13_1[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13_2[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd14[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15_2[2][ARM_JOINT_COUNT];
extern volatile float xarm_pose_pd13_2[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd11[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13_1[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd13_2[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd14[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15_2[2][ARM_JOINT_COUNT];
extern volatile float xarm_pose_pd14[2][ARM_JOINT_COUNT];
extern volatile float xarm_pose_pd14_1[2][ARM_JOINT_COUNT];
extern volatile float xarm_pose_pd14_2[2][ARM_JOINT_COUNT];
extern volatile float xarm_pose_pd15[2][ARM_JOINT_COUNT];

extern volatile uint8_t arm_joint_zero_calibration[2][5];

void Arm_TaskInit(void);
void Arm_TaskStop(void);
void Arm_Task(void *argument);
void Arm_TxTask(void *argument);
void Arm_OnFrame(const can_frame_t *frame);
bool Arm_TxQueueInit(void);

#endif



