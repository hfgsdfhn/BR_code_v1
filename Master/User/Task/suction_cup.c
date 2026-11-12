/*
 * @file suction_cup.c
 * @brief suction_cup模块的用户接口与实现。
 */

#include "suction_cup.h"

#include "cmsis_os2.h"
#include "upper.h"
#include "bsp_uart.h"
#include "STP-23L.h"
#include "my_main.h"
#include "fdcan.h"

#include <stddef.h>
#include <stdbool.h>

volatile uint8_t suction_enable;
volatile float suction_target[5];
/* 模块实现细节。 */
__ALIGNED(32) volatile float suction_feedback_angle[5];
volatile float g_up_stp23l_distance_cm;
volatile float g_down_stp23l_distance_cm;

static suction_stp23l_parser_t up_stp23l_parser;
static suction_stp23l_parser_t down_stp23l_parser;

/* 功能：读取设备参数。 */
static uint16_t SuctionCup_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/* 功能：执行 SuctionCup_KeepTrailingHeader 模块操作并维护相关状态。 */
static void SuctionCup_KeepTrailingHeader(suction_stp23l_parser_t *parser)
{
    uint16_t trailing_count = 0U;
    uint16_t index = STP23L_FRAME_SIZE;
    while (index > 0U && trailing_count < STP23L_HEADER_SIZE &&
           parser->frame[index - 1U] == STP23L_HEADER_BYTE)
    {
        --index;
        ++trailing_count;
    }
    for (index = 0U; index < trailing_count; ++index)
        parser->frame[index] = STP23L_HEADER_BYTE;
    parser->index = trailing_count;
}

/* 功能：执行 SuctionCup_IsValidFrame 模块操作并维护相关状态。 */
static uint8_t SuctionCup_IsValidFrame(const suction_stp23l_parser_t *parser)
{
    uint8_t checksum = 0U;
    uint16_t index;
    if (parser->frame[4] != STP23L_DEVICE_ADDRESS ||
        parser->frame[5] != STP23L_DISTANCE_COMMAND ||
        SuctionCup_ReadU16(&parser->frame[6]) != 0U ||
        SuctionCup_ReadU16(&parser->frame[8]) != STP23L_DATA_SIZE)
        return 0U;
    for (index = 4U; index < STP23L_CHECKSUM_OFFSET; ++index)
        checksum = (uint8_t)(checksum + parser->frame[index]);
    return (uint8_t)(checksum == parser->frame[STP23L_CHECKSUM_OFFSET]);
}

/* 功能：执行 SuctionCup_DecodeDistanceCm 模块操作并维护相关状态。 */
static float SuctionCup_DecodeDistanceCm(const suction_stp23l_parser_t *parser)
{
    uint32_t sum = 0U;
    uint32_t count = 0U;
    uint32_t index;
    for (index = 0U; index < STP23L_POINT_COUNT; ++index)
    {
        uint16_t offset = (uint16_t)(STP23L_DATA_OFFSET + index * STP23L_POINT_SIZE);
        int16_t distance_mm = (int16_t)SuctionCup_ReadU16(&parser->frame[offset]);
        uint8_t confidence = parser->frame[offset + 8U];
        if (distance_mm > 0 && confidence > 0U)
        {
            sum += (uint16_t)distance_mm;
            ++count;
        }
    }
    return (count == 0U) ? 0.0f : ((float)sum / (float)count) / 10.0f;
}

/* 功能：执行 SuctionCup_ParseBytes 模块操作并维护相关状态。 */
static void SuctionCup_ParseBytes(suction_stp23l_parser_t *parser,
                                  const uint8_t *data, uint16_t size,
                                  volatile float *distance_cm)
{
    uint16_t byte_index;
    if (parser == NULL || data == NULL || distance_cm == NULL)
        return;
    for (byte_index = 0U; byte_index < size; ++byte_index)
    {
        uint8_t byte = data[byte_index];
        if (parser->index < STP23L_HEADER_SIZE)
        {
            if (byte == STP23L_HEADER_BYTE)
                parser->frame[parser->index++] = byte;
            else
                parser->index = 0U;
            continue;
        }
        parser->frame[parser->index++] = byte;
        if (parser->index < STP23L_FRAME_SIZE)
            continue;
        if (SuctionCup_IsValidFrame(parser) != 0U)
        {
            *distance_cm = SuctionCup_DecodeDistanceCm(parser);
            parser->index = 0U;
        }
        else
        {
            SuctionCup_KeepTrailingHeader(parser);
        }
    }
}

/* 功能：执行 SuctionCup_UStp23lRx 模块操作并维护相关状态。 */
static void SuctionCup_UStp23lRx(const uint8_t *data, uint16_t size)
{
    SuctionCup_ParseBytes(&up_stp23l_parser, data, size, &g_up_stp23l_distance_cm);
}

/* 功能：执行 SuctionCup_DStp23lRx 模块操作并维护相关状态。 */
static void SuctionCup_DStp23lRx(const uint8_t *data, uint16_t size)
{
    SuctionCup_ParseBytes(&down_stp23l_parser, data, size, &g_down_stp23l_distance_cm);
}

/* 功能：初始化模块并准备外设资源。 */
void SuctionCup_Init(void)
{
    (void)BspUart_RegisterRxCallback(BSP_UART_UP_STP23L, SuctionCup_UStp23lRx);
    (void)BspUart_RegisterRxCallback(BSP_UART_DOWN_STP23L, SuctionCup_DStp23lRx);
}

/* 功能：接收并解析一帧通信数据。 */
void SuctionCup_OnFrame(const can_frame_t *frame)
{
    unsigned i;
    if (frame == NULL)
        return;
    for (i = 0U; i < 2U; i++)
    {
        if (M2006_OnFrame(&m2006_ucup[i], frame))
        {
            suction_feedback_angle[i] = m2006_ucup[i].position_rad * ARM_RAD_TO_DEG;
            return;
        }
        if (M2006_OnFrame(&m2006_dcup[i], frame))
        {
            suction_feedback_angle[i + 2U] = m2006_dcup[i].position_rad * ARM_RAD_TO_DEG;
            return;
        }
    }
    if (M3508_OnFrame(&m3508_dcup[0], frame))
        suction_feedback_angle[4] = m3508_dcup[0].position_rad * ARM_RAD_TO_DEG;
}

/* 功能：发送数据或控制指令。 */
static void SuctionCup_SendGroupFrames(void)
{
    can_frame_t frame;
    int16_t can2_current[4] = {0};
    int16_t can3_current[4] = {0};
    int16_t can2_high_current[4] = {0};

    /* 模块实现细节。 */
    can2_current[0] = M2006_Calc(&m2006_larm, 0.002f);
    can2_high_current[0] = M3508_Calc(&m3508_dcup[0], 0.002f);
    can2_current[1] = M2006_Calc(&m2006_dcup[0], 0.002f);
    can2_current[2] = M2006_Calc(&m2006_dcup[1], 0.002f);
    M2006_BuildGroupFrame(1U, can2_current, &frame);
    frame.hfdcan = &hfdcan2;
    (void)BspCan_Send(&frame);

    /* 模块实现细节。 */
    M3508_BuildGroupFrame(5U, can2_high_current, &frame);
    frame.hfdcan = &hfdcan2;
    (void)BspCan_Send(&frame);

    /* 模块实现细节。 */
    can3_current[1] = M2006_Calc(&m2006_ucup[0], 0.002f);
    can3_current[2] = M2006_Calc(&m2006_ucup[1], 0.002f);
    M2006_BuildGroupFrame(1U, can3_current, &frame);
    frame.hfdcan = &hfdcan3;
    (void)BspCan_Send(&frame);
}

/* 功能：执行周期性服务并维护运行状态。 */
void SuctionCup_Service(void)
{
    unsigned i;
    bool enabled = suction_enable != 0U;
    for (i = 0U; i < 2U; i++)
    {
        M2006_SetTargetDeferred(&m2006_ucup[i], enabled ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                enabled ? suction_target[i] : 0.0f);
        M2006_SetTargetDeferred(&m2006_dcup[i], enabled ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                enabled ? suction_target[i + 2U] : 0.0f);
    }
    M3508_SetTargetDeferred(&m3508_dcup[0], enabled ? M3508_MODE_POSITION : M3508_MODE_STOP,
                            enabled ? suction_target[4] : 0.0f);
    SuctionCup_SendGroupFrames();
}
/* 功能：执行周期任务主流程。 */
void SuctionCup_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        SuctionCup_Service();
        osDelay((osKernelGetTickFreq() * 2U + 999U) / 1000U);
    }
}
