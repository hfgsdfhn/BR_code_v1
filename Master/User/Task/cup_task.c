#include "cup_task.h"

#include "../Mechanism/Chassis/chassis.h"
#include "../Mechanism/Upper/cup_config.h"
#include "STP-23L.h"
#include "arm_task.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "my_main.h"
#include "status_task.h"
#include "upper.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

volatile uint8_t suction_enable = 1U;
volatile float suction_target[2][3];
static volatile uint8_t pd13_stp_state;
static bool pd13_stp_started[2];
static bool pd13_stp_done[2];
static bool pd13_position_second_group;
static bool id3_wait_for_id2;
static float id3_hold_target[2];
static bool id3_id2_released[2];
typedef enum
{
    CUP_ACT_NONE,
    CUP_ACT_PD13,
    CUP_ACT_PD14,
    CUP_ACT_PD15,
    CUP_ACT_PA8,
    CUP_ACT_PA11,
    CUP_ACT_PA12
} cup_act_t;
static cup_act_t cup_act;
static uint8_t cup_act_step;
static uint8_t pd13_press_step, pd14_press_step, pd15_press_step, pa8_press_step, pa11_press_step, pa12_press_step, k0_press_step;
static void Cup_ServiceAction(void);

volatile float pd13_stp_normal_speed_rad_s = 20.0f;
volatile float pd13_stp_decel_speed_rad_s = 8.0f;
volatile float pd13_stp_decel_distance_cm = 10.0f;
volatile float pd13_stp_upper_stop_distance_cm = 1.0f;
volatile float pd13_stp_lower_stop_distance_cm = 2.5f;

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
    while (index > 0U && trailing_count < STP23L_HEADER_SIZE && parser->frame[index - 1U] == STP23L_HEADER_BYTE)
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
    if (parser->frame[4] != STP23L_DEVICE_ADDRESS || parser->frame[5] != STP23L_DISTANCE_COMMAND ||
        SuctionCup_ReadU16(&parser->frame[6]) != 0U || SuctionCup_ReadU16(&parser->frame[8]) != STP23L_DATA_SIZE)
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

static void SuctionCup_ParseBytes(suction_stp23l_parser_t *parser, const uint8_t *data, uint16_t size,
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

    for (i = 0U; i < 2U; ++i)
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

static void SuctionCup_ServicePd13StpLink(void)
{
    m2006_t *motors[2] = {&m2006_ucup[0], &m2006_dcup[0]};
    const volatile float *distances[2] = {&ustp23l_distance_cm, &dstp23l_distance_cm};
    unsigned side;
    if (pd13_stp_state == 1U)
    {
        float up_id3 = suction_feedback_angle[ucup][1U];
        float down_id3 = suction_feedback_angle[dcup][1U];
        if (up_id3 >= 80.0f && up_id3 <= 100.0f && down_id3 >= 80.0f && down_id3 <= 100.0f)
            pd13_stp_state = 2U;
        return;
    }
    if (pd13_stp_state != 2U)
        return;
    for (side = 0U; side < 2U; ++side)
    {
        m2006_t *motor = motors[side];
        float distance = *distances[side];
        if (!motor->feedback_online || !M2006_IsSoftwareZeroCalibrated(motor))
            continue;

        float position_rad = motor->position_rad;
        float position_limit = motor->max_output_angle_deg;
        if (!pd13_stp_started[side] && distance >= 0.0f && distance <= 25.0f)
            pd13_stp_started[side] = true;
        if (!pd13_stp_started[side] || pd13_stp_done[side])
            continue;
        if (distance <= (side == 0U ? pd13_stp_upper_stop_distance_cm : pd13_stp_lower_stop_distance_cm) ||
            position_rad >= position_limit * M2006_DEG_TO_RAD)
        {
            float hold_target = M2006_ClampPositionDegrees(motor, position_rad * ARM_RAD_TO_DEG);
            M2006_SetTargetDeferred(motor, M2006_MODE_POSITION, hold_target);
            suction_target[side == 0U ? ucup : dcup][0U] = hold_target;
            pd13_stp_done[side] = true;
        }
        else
        {
            float speed =
                (distance <= pd13_stp_decel_distance_cm ||
                 position_rad >= (position_limit - pd13_stp_decel_distance_cm) * M2006_DEG_TO_RAD)
                    ? pd13_stp_decel_speed_rad_s
                    : pd13_stp_normal_speed_rad_s;
            M2006_SetTargetDeferred(motor, M2006_MODE_SPEED, speed);
        }
    }
    if (pd13_stp_done[0] && pd13_stp_done[1])
        pd13_stp_state = 3U;
}
void SuctionCup_Service(void)
{
    unsigned i;
    bool enabled;

    SuctionCup_MotorWatchdog();
    enabled = suction_enable != 0U;

    for (i = 0U; i < 2U; i++)
    {
        if (i == 1U)
        {
            float up_rotation_target = suction_target[ucup][1U];
            float down_rotation_target = suction_target[dcup][1U];
            if (id3_wait_for_id2)
            {
                if (suction_feedback_angle[ucup][0U] > 300.0f || suction_feedback_angle[dcup][0U] > 300.0f)
                {
                    id3_id2_released[ucup] = true;
                    id3_id2_released[dcup] = true;
                }
                if (!id3_id2_released[ucup])
                    up_rotation_target = id3_hold_target[ucup];
                if (!id3_id2_released[dcup])
                    down_rotation_target = id3_hold_target[dcup];
            }
            m2006_ucup[1].trajectory_time_s =
                ((up_rotation_target > 100.0f ||
                  (up_rotation_target < 10.0f && suction_feedback_angle[ucup][1U] > 100.0f))
                     ? BR_M2006_CUP1_ROTATION_TRAJECTORY_TIME_S
                     : BR_M2006_CUP1_TRAJECTORY_TIME_S);
            m2006_dcup[1].trajectory_time_s =
                ((down_rotation_target > 100.0f ||
                  (down_rotation_target < 10.0f && suction_feedback_angle[dcup][1U] > 100.0f))
                     ? BR_M2006_CUP1_ROTATION_TRAJECTORY_TIME_S
                     : BR_M2006_CUP1_TRAJECTORY_TIME_S);
        }
        bool up_ready = m2006_ucup[i].feedback_online;
        bool down_ready = m2006_dcup[i].feedback_online;

        if (!(i == 0U && (pd13_stp_state == 1U || pd13_stp_state == 2U) && pd13_stp_started[0] && !pd13_stp_done[0]))
            M2006_SetTargetDeferred(&m2006_ucup[i], (enabled && up_ready) ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                    (enabled && up_ready) ? ((i == 1U && id3_wait_for_id2 && !id3_id2_released[ucup])
                                                                 ? id3_hold_target[ucup]
                                                                 : suction_target[ucup][i])
                                                          : 0.0f);
        if (!(i == 0U && (pd13_stp_state == 1U || pd13_stp_state == 2U) && pd13_stp_started[1] && !pd13_stp_done[1]))
            M2006_SetTargetDeferred(&m2006_dcup[i], (enabled && down_ready) ? M2006_MODE_POSITION : M2006_MODE_STOP,
                                    (enabled && down_ready) ? ((i == 1U && id3_wait_for_id2 && !id3_id2_released[dcup])
                                                                   ? id3_hold_target[dcup]
                                                                   : suction_target[dcup][i])
                                                            : 0.0f);
    }
    M3508_SetTargetDeferred(&m3508_dcup[0],
                            (enabled && m3508_dcup[0].feedback_online) ? M3508_MODE_POSITION : M3508_MODE_STOP,
                            (enabled && m3508_dcup[0].feedback_online) ? suction_target[dcup][2U] : 0.0f);

    SuctionCup_ServicePd13StpLink();
    Cup_ServiceAction();
    SuctionCup_SendGroupFrames();
}

static void Cup_ResetAction(void)
{
    cup_act = CUP_ACT_NONE;
    cup_act_step = 0U;
    pd13_stp_state = 0U;
    pd13_stp_started[0] = false;
    pd13_stp_started[1] = false;
    pd13_stp_done[0] = false;
    pd13_stp_done[1] = false;
    id3_wait_for_id2 = false;
}
static void Cup_SetTarget(float ux, float dx, float y)
{
    suction_target[ucup][0U] = ux;
    suction_target[dcup][0U] = dx;
    suction_target[ucup][1U] = y;
    suction_target[dcup][1U] = y;
}
static bool Cup_Reached(float ux, float dx, float y)
{
    return fabsf(suction_feedback_angle[ucup][0U] - ux) <= 5.0f &&
           fabsf(suction_feedback_angle[dcup][0U] - dx) <= 5.0f &&
           fabsf(suction_feedback_angle[ucup][1U] - y) <= 5.0f && fabsf(suction_feedback_angle[dcup][1U] - y) <= 5.0f;
}
static void Cup_ServiceAction(void)
{
    if (cup_act == CUP_ACT_NONE)
        return;
    if (cup_act == CUP_ACT_PA12)
    {
        if (cup_act_step == 0U && m3508_dcup[0].feedback_online)
        {
            float m3508_angle = m3508_dcup[0].position_rad * ARM_RAD_TO_DEG;
            if (m3508_angle > 380.0f && m3508_angle < 400.0f)
            {
                gpio_pb6 = 1U;
                gpio_pd4 = 0U;
                suction_target[dcup][0U] = BR_M2006_LOWER_CUP_ID2_MAX_DEG;
                cup_act_step = 1U;
            }
        }
        return;
    }
    if (cup_act == CUP_ACT_PA8)
    {
        if (cup_act_step == 2U && suction_feedback_angle[dcup][2U] > 200.0f && suction_feedback_angle[dcup][2U] < 230.0f)
        {
            Cup_SetTarget(125.0f, 125.0f, 190.0f);
            cup_act_step = 3U;
        }
        return;
    }
    if (cup_act == CUP_ACT_PD13)
    {
        if (cup_act_step == 0U && suction_feedback_angle[dcup][2U] >= 0.0f && suction_feedback_angle[dcup][2U] < 20.0f)
        {
            Cup_SetTarget(125.0f, 125.0f, 100.0f);
            pd13_stp_state = 1U;
            cup_act_step = 1U;
        }
        else if (cup_act_step == 1U && pd13_stp_state == 3U)
        {
            cup_act_step = 3U;
        }
        if (cup_act_step == 2U && suction_feedback_angle[dcup][2U] > 200.0f &&
                 suction_feedback_angle[dcup][2U] < 230.0f)
        {
            Cup_SetTarget(125.0f, 125.0f, 190.0f);
            pd13_stp_state = 0U;
            pd13_stp_started[0] = false;
            pd13_stp_started[1] = false;
            pd13_stp_done[0] = false;
            pd13_stp_done[1] = false;
            cup_act_step = 3U;
        }
        return;
    }
    {
        bool done;
        if (cup_act == CUP_ACT_PD14)
            done = Cup_Reached(880.0f, 830.0f, 190.0f);
        else if (cup_act == CUP_ACT_PD15)
            done = Cup_Reached(880.0f, 830.0f, 10.0f);
        else if (cup_act == CUP_ACT_PA11)
            done = Cup_Reached(125.0f, 125.0f, 100.0f);
        else
            done = cup_act_step != 0U ? Cup_Reached(880.0f, 830.0f, 100.0f) : Cup_Reached(125.0f, 125.0f, 100.0f);
        if (done)
            cup_act = CUP_ACT_NONE;
    }
}

static void SuctionCup_ProcessRemote(const send_to_cup_t *status)
{
    static uint16_t previous_keys;
    static bool previous_keys_valid;
    static bool previous_pa5;
    uint16_t keys;
    uint16_t pressed;
    bool pa5;

    if (status == NULL)
        return;

    if ((status->remote.switch_state & (1U << 0U)) == 0U)
    {
        previous_keys_valid = false;
        Cup_ResetAction();
        return;
    }

    if ((status->remote.switch_state & (1U << 1U)) != 0U)
    {
        previous_keys_valid = false;
        return;
    }

    keys = (uint16_t)status->remote.key[0];
    keys |= (uint16_t)((status->remote.key[1] & 1U) << 8U);

    if (!previous_keys_valid)
    {
        previous_keys = keys;
        previous_keys_valid = true;
        return;
    }

    pressed = (uint16_t)(keys & (uint16_t)~previous_keys);
    pa5 = ((status->remote.switch_state & (1U << 1U)) != 0U);
    if (pa5 != previous_pa5)
        Cup_ResetAction();
    previous_pa5 = pa5;

    if ((pressed & (1U << 2U)) != 0U)
        suction_enable ^= 1U;

    if ((pressed & (1U << 0U)) != 0U)
    {
        Cup_ResetAction();
        k0_press_step ^= 1U;
        if (k0_press_step != 0U)
        {
            gpio_pd7 = 1U;
            gpio_pb6 = 1U;
            gpio_pb7 = 0U;
            gpio_pd4 = 0U;
        }
        else
        {
            gpio_pd7 = 0U;
            gpio_pb6 = 0U;
            gpio_pb7 = 1U;
            gpio_pd4 = 1U;
        }
    }

    if ((pressed & (1U << 1U)) != 0U)
    {
        Cup_ResetAction();
        k0_press_step = 0U;
        pd13_press_step = 0U;
        pd14_press_step = 0U;
        pd15_press_step = 0U;
        pa8_press_step = 0U;
        pa11_press_step = 0U;
        pa12_press_step = 0U;
        gpio_pd7 = 0U;
        gpio_pb6 = 0U;
        gpio_pb7 = 0U;
        gpio_pd4 = 0U;
        gpio_pd3 = 0U;
        suction_target[dcup][2U] = 0.0f;
        Cup_SetTarget(125.0f, 125.0f, 100.0f);
    }

    if ((pressed & (1U << 3U)) != 0U)
    {
        bool pd13_first_press = (pd13_press_step == 0U);

        Cup_ResetAction();
        pd14_press_step = 0U;
        pd15_press_step = 0U;
        k0_press_step = 0U;
        pa8_press_step = 0U;
        pa12_press_step = 0U;
        cup_act = CUP_ACT_PD13;
        pd13_press_step = pd13_first_press ? 1U : 0U;
        pa8_press_step = 0U;
        pa12_press_step = 0U;
        gpio_pd7 = 1U;
        gpio_pb6 = 1U;
        gpio_pb7 = 0U;
        gpio_pd4 = 0U;

        if (pd13_first_press)
        {
            gpio_pd3 = 0U;
            suction_target[dcup][2U] = 0.0f;
            cup_act_step = 0U;
        }
        else
        {
            gpio_pd3 = 1U;
            suction_target[dcup][2U] = 220.0f;
            cup_act_step = 2U;
        }
    }

    if ((pressed & (1U << 4U)) != 0U)
    {
        bool second_press = (pd14_press_step != 0U);
        pd14_press_step = second_press ? 0U : 1U;
        Cup_ResetAction();
        pa8_press_step = 0U;
        pa12_press_step = 0U;
        gpio_pd3 = second_press ? 0U : 1U;
        suction_target[dcup][2U] = second_press ? 0.0f : 220.0f;
        if (second_press)
        {
            gpio_pd7 = 0U;
            gpio_pb6 = 0U;
            gpio_pb7 = 1U;
            gpio_pd4 = 1U;
        }
        Cup_SetTarget(880.0f, 830.0f, 190.0f);
        k0_press_step = 0U;
        if (!second_press)
        {
            cup_act = CUP_ACT_PD14;
        }
    }

    if ((pressed & (1U << 5U)) != 0U)
    {
        bool second_press = (pd15_press_step != 0U);
        pd15_press_step = second_press ? 0U : 1U;
        Cup_ResetAction();
        pa8_press_step = 0U;
        pa12_press_step = 0U;
        gpio_pd3 = second_press ? 0U : 1U;
        suction_target[dcup][2U] = second_press ? 0.0f : 220.0f;
        if (second_press)
        {
            gpio_pd7 = 0U;
            gpio_pb6 = 0U;
            gpio_pb7 = 1U;
            gpio_pd4 = 1U;
        }
        Cup_SetTarget(880.0f, 830.0f, 10.0f);
        k0_press_step = 0U;
        if (!second_press)
        {
            cup_act = CUP_ACT_PD15;
        }
    }

    if ((pressed & (1U << 6U)) != 0U)
    {
        bool first_press = (pa8_press_step == 0U);
        pa8_press_step = first_press ? 1U : 0U;
        Cup_ResetAction();
        pd14_press_step = 0U;
        pd15_press_step = 0U;
        k0_press_step = 0U;
        pd13_press_step = 0U;
        pa12_press_step = 0U;
        cup_act = CUP_ACT_PA8;
        cup_act_step = first_press ? 1U : 2U;
        gpio_pd7 = 1U;
        gpio_pb6 = 1U;
        gpio_pb7 = 0U;
        gpio_pd4 = 0U;
        gpio_pd3 = first_press ? 0U : 1U;
        suction_target[dcup][2U] = first_press ? 0.0f : 220.0f;
        if (first_press)
        {
            Cup_SetTarget(880.0f, 830.0f, 100.0f);
        }
    }

    if ((pressed & (1U << 7U)) != 0U)
    {
        bool pa11_first_press = (pa11_press_step == 0U);
        pa11_press_step = pa11_first_press ? 1U : 0U;
        Cup_ResetAction();
        k0_press_step = pa11_first_press ? 1U : 0U;
        pa12_press_step = 0U;
        cup_act = CUP_ACT_PA11;
        cup_act_step = pa11_first_press ? 0U : 1U;
        if (pa11_first_press)
        {
            set_target_position(305.0f, 0.0f, 9.6f);
            gpio_pd7 = 1U;
            gpio_pb6 = 1U;
            gpio_pb7 = 0U;
            gpio_pd4 = 0U;
            gpio_pd3 = 0U;
            suction_target[dcup][2U] = 350.0f;
            Cup_SetTarget(125.0f, 125.0f, 100.0f);
        }
        else
        {
            set_target_position(0.0f, 0.0f, 0.0f);
            gpio_pd7 = 0U;
            gpio_pb6 = 0U;
            gpio_pb7 = 0U;
            gpio_pd4 = 0U;
            gpio_pd3 = 0U;
            suction_target[dcup][2U] = 0.0f;
            Cup_SetTarget(125.0f, 125.0f, 100.0f);
        }
    }

    if ((pressed & (1U << 8U)) != 0U)
    {
        pa12_press_step = (uint8_t)((pa12_press_step + 1U) % 2U);
        Cup_ResetAction();
        pd14_press_step = 0U;
        pd15_press_step = 0U;
        k0_press_step = 0U;
        pd13_press_step = 0U;
        pa8_press_step = 0U;
        cup_act = CUP_ACT_PA12;
        cup_act_step = pa12_press_step == 0U ? 2U : 0U;
        if (pa12_press_step != 0U)
        {
            suction_target[ucup][0U] = 125.0f;
            suction_target[dcup][0U] = 125.0f;
            suction_target[ucup][1U] = 100.0f;
            suction_target[dcup][1U] = 100.0f;
            suction_target[dcup][2U] = 400.0f;
        }
        else
        {
            suction_target[dcup][0U] = 125.0f;
            suction_target[dcup][1U] = 100.0f;
        }
    }

    previous_keys = keys;
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
