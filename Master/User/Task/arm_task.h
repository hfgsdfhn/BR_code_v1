/*
 * @file arm_task.h
 * @brief arm_task妯″潡鐨勭敤鎴锋帴鍙ｄ笌瀹炵幇銆?
 */

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

extern volatile float arm_pose_pd13[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd14[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd14_2[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd15[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd11[2][ARM_JOINT_COUNT];
extern volatile uint8_t arm_joint_zero_calibration[2][5];

/* 鍔熻兘锛氬垵濮嬪寲Arm_TaskInit瀵瑰簲鐨勫璁俱€佸弬鏁板拰杩愯鐘舵€併€?*/
void Arm_TaskInit(void);
/* 鍔熻兘锛氬仠姝rm_TaskStop瀵瑰簲鐨勮緭鍑哄苟娓呯悊杩愯鐘舵€併€?*/
void Arm_TaskStop(void);
/* 鍔熻兘锛氭墽琛孉rm_Task瀵瑰簲鐨勫懆鏈熶换鍔″苟缁存姢杩愯鐘舵€併€?*/
void Arm_Task(void *argument);
/* 鍔熻兘锛氭墽琛孉rm_TxTask瀵瑰簲鐨勫懆鏈熶换鍔″苟缁存姢杩愯鐘舵€併€?*/
void Arm_TxTask(void *argument);
/* 鍔熻兘锛氭帴鏀跺苟瑙ｆ瀽Arm_OnFrame瀵瑰簲鐨勮緭鍏ユ暟鎹紝鏇存柊妯″潡鐘舵€併€?*/
void Arm_OnFrame(const can_frame_t *frame);
/* 鍔熻兘锛氬皢Arm_TxQueueInit瀵瑰簲鐨勭洰鏍囨暟鎹姞鍏ユ墽琛岄槦鍒椼€?*/
bool Arm_TxQueueInit(void);

#endif
