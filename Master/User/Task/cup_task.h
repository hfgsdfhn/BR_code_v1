/*
 * @file suction_cup.h
 * @brief suction_cup模块的用户接口与实现。
 */

#ifndef SUCTION_CUP_H
#define SUCTION_CUP_H
#include <stdint.h>
#include <stdbool.h>
#include "STP-23L.h"
#include "../Driver/Motor/DJ_M2006/m2006.h"
#include "../Driver/Motor/DJ_M3508/m3508.h"
#include "../Peripheral/CAN/bsp_can.h"

typedef struct
{
    uint8_t frame[STP23L_FRAME_SIZE];
    uint16_t index; /* 元素索引。 */
} suction_stp23l_parser_t;

extern m2006_t m2006_ucup[2];
extern m2006_t m2006_dcup[2];
extern m3508_t m3508_dcup[1];

extern volatile uint8_t suction_enable;
extern volatile float suction_target[2][3];

extern volatile float suction_feedback_angle[2][3];
extern volatile float g_up_stp23l_distance_cm;
extern volatile float g_down_stp23l_distance_cm;

/* 功能：初始化SuctionCup_Init对应的外设、参数和运行状态。 */
void SuctionCup_Init(void);
/* 功能：接收并解析SuctionCup_OnFrame对应的输入数据，更新模块状态。 */
void SuctionCup_OnFrame(const can_frame_t *frame);
/* 功能：执行SuctionCup_Service对应的周期任务并维护运行状态。 */
void SuctionCup_Service(void);
/* 功能：执行SuctionCup_Task对应的周期任务并维护运行状态。 */
void SuctionCup_Task(void *argument);
#endif
