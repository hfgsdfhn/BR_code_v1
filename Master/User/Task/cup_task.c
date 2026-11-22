
#include "cup_task.h"


#include "cmsis_os2.h"
#include "upper.h"
#include "bsp_uart.h"
#include "STP-23L.h"
#include "my_main.h"
#include "fdcan.h"
#include "status_task.h"
#include "arm_task.h"

#include <stddef.h>
#include <stdbool.h>

volatile uint8_t suction_enable = 0U;
volatile float suction_target[2][3];

volatile uint32_t cup_motor_offline_count = 0U;
volatile uint32_t cup_motor_recover_count = 0U;
static bool cup_motor_online_prev[5];

__ALIGNED(32) volatile float suction_feedback_angle[2][3];
volatile float ustp23l_distance_cm;
volatile float dstp23l_distance_cm;

static suction_stp23l_parser_t up_stp23l_parser;
static suction_stp23l_parser_t down_stp23l_parser;

static uint16_t SuctionCup_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

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

void UStp23lRx(const uint8_t *data, uint16_t size)
{
    SuctionCup_ParseBytes(&up_stp23l_parser, data, size, &ustp23l_distance_cm);
}

void DStp23lRx(const uint8_t *data, uint16_t size)
{
    SuctionCup_ParseBytes(&down_stp23l_parser, data, size, &dstp23l_distance_cm);
}

void SuctionCup_Init(void)
{
    (void)BspUart_RegisterRxCallback(BSP_UART_UP_STP23L, UStp23lRx);
    (void)BspUart_RegisterRxCallback(BSP_UART_DOWN_STP23L, DStp23lRx);
}

void SuctionCup_OnFrame(const can_frame_t *frame)
{
    unsigned i;
    if (frame == NULL)
        return;

    if (M3508_OnFrame(&m3508_dcup[0], frame))
    {
        suction_feedback_angle[dcup][2U] = m3508_dcup[0].position_rad * ARM_RAD_TO_DEG;
        return;
    }

    for (i = 0U; i < 2U; i++)
    {
        if (M2006_OnFrame(&m2006_ucup[i], frame))
        {
            suction_feedback_angle[ucup][i] = m2006_ucup[i].position_rad * ARM_RAD_TO_DEG;
            return;
        }
        if (M2006_OnFrame(&m2006_dcup[i], frame))
        {
            suction_feedback_angle[dcup][i] = m2006_dcup[i].position_rad * ARM_RAD_TO_DEG;
            return;
        }
    }
}

static void SuctionCup_SendGroupFrames(void)
{
    can_frame_t frame;
    int16_t can2_current[4] = {0};
    int16_t can2_high_current[4] = {0};
    int16_t can3_current[4] = {0};

    can2_current[0] = M2006_Calc(&m2006_larm, 0.002f);
    can2_current[1] = M2006_Calc(&m2006_dcup[0], 0.002f);
    can2_current[2] = M2006_Calc(&m2006_dcup[1], 0.002f);
    M2006_BuildGroupFrame(1U, can2_current, &frame);
    frame.hfdcan = &hfdcan2;
    (void)BspCan_Send(&frame);

    can2_high_current[0] = M3508_Calc(&m3508_dcup[0], 0.002f);
    M3508_BuildGroupFrame(5U, can2_high_current, &frame);
    frame.hfdcan = &hfdcan2;
    (void)BspCan_Send(&frame);

    can3_current[1] = M2006_Calc(&m2006_ucup[0], 0.002f);
    can3_current[2] = M2006_Calc(&m2006_ucup[1], 0.002f);
    M2006_BuildGroupFrame(1U, can3_current, &frame);
    frame.hfdcan = &hfdcan3;
    (void)BspCan_Send(&frame);
}
static void SuctionCup_MotorWatchdog(void)
{
    bool online_now[5];
    unsigned i;

    for (i = 0U; i < 2U; i++)
    {
        if (m2006_ucup[i].feedback_online)
        {
            if (m2006_ucup[i].feedback_age_cycles < M2006_FEEDBACK_TIMEOUT_CYCLES)
            {
                m2006_ucup[i].feedback_age_cycles++;
            }
            else
            {
                m2006_ucup[i].feedback_online = false;
            }
        }
        if (m2006_dcup[i].feedback_online)
        {
            if (m2006_dcup[i].feedback_age_cycles < M2006_FEEDBACK_TIMEOUT_CYCLES)
            {
                m2006_dcup[i].feedback_age_cycles++;
            }
            else
            {
                m2006_dcup[i].feedback_online = false;
            }
        }
        online_now[i] = m2006_ucup[i].feedback_online;
        online_now[i + 2U] = m2006_dcup[i].feedback_online;
    }
    if (m3508_dcup[0].feedback_online)
    {
        if (m3508_dcup[0].feedback_age_cycles < M3508_FEEDBACK_TIMEOUT_CYCLES)
        {
            m3508_dcup[0].feedback_age_cycles++;
        }
        else
        {
            m3508_dcup[0].feedback_online = false;
        }
    }
    online_now[4] = m3508_dcup[0].feedback_online;

    for (i = 0U; i < 5U; i++)
    {
        if (cup_motor_online_prev[i] && !online_now[i])
        {
            cup_motor_offline_count++;
        }
        else if (!cup_motor_online_prev[i] && online_now[i])
        {
            cup_motor_recover_count++;
        }
        cup_motor_online_prev[i] = online_now[i];
    }
}

static bool SuctionCup_StartupZeroCalibrationComplete(void)
{
    unsigned i;

    for (i = 0U; i < 2U; i++)
    {
        if (!m2006_ucup[i].received || !m2006_dcup[i].received)
            return false;
    }
    return m3508_dcup[0].received;
}
void SuctionCup_Service(void)
{
    unsigned i;
    bool enabled;

    SuctionCup_MotorWatchdog();


    enabled = suction_enable != 0U;

    for (i = 0U; i < 2U; i++)
    {
        bool up_ready = m2006_ucup[i].feedback_online;
        bool down_ready = m2006_dcup[i].feedback_online;

        M2006_SetTargetDeferred(&m2006_ucup[i],
                                (enabled && up_ready) ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                (enabled && up_ready) ? suction_target[ucup][i] : 0.0f);
        M2006_SetTargetDeferred(&m2006_dcup[i],
                                (enabled && down_ready) ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                (enabled && down_ready) ? suction_target[dcup][i] : 0.0f);
    }
    M3508_SetTargetDeferred(&m3508_dcup[0],
                            (enabled && m3508_dcup[0].feedback_online) ? M3508_MODE_POSITION : M3508_MODE_STOP,
                            (enabled && m3508_dcup[0].feedback_online) ? suction_target[dcup][2U] : 0.0f);
    SuctionCup_SendGroupFrames();
}
static void SuctionCup_ProcessRemote(const send_to_cup_t *status)
{
    static uint16_t previous_keys;
    static bool previous_keys_valid;
    static bool pd14_second_press;
    static bool pd15_second_press;
    uint16_t current_keys;
    uint16_t pressed_keys;
    if (status == NULL)
        return;
    if ((status->remote.switch_state & (1U << 0)) == 0U)
    {
        previous_keys = 0U;
        previous_keys_valid = false;
        pd14_second_press = false;
        pd15_second_press = false;
        return;
    }
    current_keys = (uint16_t)(status->remote.key[0] & ((1U << 0) | (1U << 1) | (1U << 2) | (1U << 3) | (1U << 4) | (1U << 5) | (1U << 6)));
    if (!previous_keys_valid)
    {
        previous_keys = current_keys;
        previous_keys_valid = true;
        return;
    }
    pressed_keys = (uint16_t)(current_keys & (uint16_t)~previous_keys);
    if ((pressed_keys & (1U << 0)) != 0U)
        suction_enable ^= 1U;
    if ((pressed_keys & (1U << 1)) != 0U)
    {
        gpio_pd7 ^= 1U;
        gpio_pb6 ^= 1U;
    }
    if ((pressed_keys & (1U << 2)) != 0U)
    {
        gpio_pb7 ^= 1U;
        gpio_pd4 ^= 1U;
    }
    if ((pressed_keys & (1U << 3)) != 0U)
        gpio_pd1 ^= 1U;
    if ((pressed_keys & (1U << 6)) != 0U)
        suction_target[dcup][2U] = (suction_target[dcup][2U] == 220.0f) ? 0.0f : 220.0f;
    if ((pressed_keys & (1U << 4)) != 0U)
    {
        if (!pd14_second_press)
        {
            suction_target[ucup][0U] = 890.0f;
            suction_target[dcup][0U] = 700.0f;
        }
        else
        {
            suction_target[ucup][0U] = 100.0f;
            suction_target[dcup][0U] = 0.0f;
        }
        suction_target[ucup][1U] = 90.0f;
        suction_target[dcup][1U] = 90.0f;
        pd14_second_press = !pd14_second_press;
        pd15_second_press = false;
    }
    if ((pressed_keys & (1U << 5)) != 0U)
    {
        if (!pd15_second_press)
        {
            suction_target[ucup][0U] = 890.0f;
            suction_target[dcup][0U] = 790.0f;
        }
        else
        {
            suction_target[ucup][0U] = 600.0f;
            suction_target[dcup][0U] = 500.0f;
        }
        suction_target[ucup][1U] = 0.0f;
        suction_target[dcup][1U] = 0.0f;
        pd15_second_press = !pd15_second_press;
        pd14_second_press = false;
    }
    previous_keys = current_keys;
}

void SuctionCup_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        send_to_cup_t status;
        if (osMessageQueueGet(Cup_QueueHandle, &status, NULL, 0U) == osOK)
            SuctionCup_ProcessRemote(&status);
        SuctionCup_Service();
        osDelay((osKernelGetTickFreq() * 2U + 999U) / 1000U);
    }
}
