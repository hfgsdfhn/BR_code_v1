/*
 * @file suction_cup.h
 * @brief suction_cup模块的用户接口与实现。
 */

#ifndef SUCTION_CUP_H
#define SUCTION_CUP_H
#include <stdint.h>
#include "STP-23L.h"
#include "../Driver/Motor/DJ_M2006/m2006.h"
#include "../Driver/Motor/DJ_M3508/m3508.h"
#include "../Peripheral/CAN/bsp_can.h"

typedef struct
{
    uint8_t frame[STP23L_FRAME_SIZE]; /* 数据缓冲区或数据帧。 */
    uint16_t index; /* 元素索引。 */
} suction_stp23l_parser_t;

extern m2006_t m2006_ucup[2]; /* 成员变量。 */
extern m2006_t m2006_dcup[2]; /* 成员变量。 */
extern m3508_t m3508_dcup[1]; /* 成员变量。 */

extern volatile uint8_t suction_enable; /* 状态标志。 */
extern volatile float suction_target[5]; /* 成员变量。 */
/* 代码说明：相关模块实现。 */
extern volatile float suction_feedback_angle[5]; /* 成员变量。 */
extern volatile float g_up_stp23l_distance_cm; /* 成员变量。 */
extern volatile float g_down_stp23l_distance_cm; /* 成员变量。 */

/* 功能：执行 SuctionCup_Init 函数对应的模块操作。 */
void SuctionCup_Init(void);
/* 功能：执行 SuctionCup_OnFrame 函数对应的模块操作。 */
void SuctionCup_OnFrame(const can_frame_t *frame);
/* 功能：执行 SuctionCup_Service 函数对应的模块操作。 */
void SuctionCup_Service(void);
/* 功能：执行 SuctionCup_Task 函数对应的模块操作。 */
void SuctionCup_Task(void *argument);
#endif
