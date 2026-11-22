
#include "rs01.h"
#include "bsp_can.h"
#include "my_main.h"
#include "quintic_planner.h"
#include "rs05.h"
#include "trapezoid_planner.h"
#include <math.h>
#include <string.h>

volatile float rs01_trajectory_max_speed_rad_s = 32.9867f;
volatile float rs01_trajectory_max_acceleration_rad_s2 = 30.0f;

volatile float rs01_trajectory_time_s = 0.0f;

volatile float rs01_speed_max_acceleration_rad_s2 = 30.0f;

volatile float rs01_speed_limit_current_a = 23.0f;
volatile float rs01_speed_accel_rad_s2 = 30.0f;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t enc(float v, float lo, float hi)
{
    v = clampf(v, lo, hi);
    return (uint16_t)((v - lo) * 65535.0f / (hi - lo));
}

static float dec(uint16_t v, float lo, float hi)
{
    return lo + (float)v * (hi - lo) / 65535.0f;
}

static uint16_t rdbe(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t pack(uint8_t cmd, uint16_t extra, uint8_t id)
{
    return ((uint32_t)cmd << 24) | ((uint32_t)extra << 8) | id;
}

static void RS01_PlanTrajectory(rs01_t *m, float start, float target)
{
    quintic_plan_t plan;
    float duration_ms;

    Quintic_SetDefaultLimits(m->recovery_trajectory ? (rs01_trajectory_max_speed_rad_s * 0.3333333333f) : rs01_trajectory_max_speed_rad_s, rs01_trajectory_max_acceleration_rad_s2,
                             (float)RS01_TRAJECTORY_PERIOD_MS * 0.001f);
    Quintic_Plan(&plan, start, target, m->velocity, Quintic_DefaultDuration(target - start, m->velocity, m->trajectory_time_s));
    m->trajectory_start_position = start;
    m->trajectory_delta_position = target - start;
    m->trajectory_k0 = plan.k0;
    duration_ms = plan.duration * 1000.0f;
    if (duration_ms > 0.0f)
    {
        duration_ms = ceilf(duration_ms / (float)RS01_TRAJECTORY_PERIOD_MS) * (float)RS01_TRAJECTORY_PERIOD_MS;
    }
    m->trajectory_duration_ms = (uint32_t)duration_ms;
}

static void RS01_BeginPositionTrajectory(rs01_t *m, uint32_t now)
{
    RS01_PlanTrajectory(m, m->position, m->target);
    m->trajectory_start_tick = now;
    m->reference_position = m->trajectory_start_position;
    m->reference_velocity = 0.0f;
    m->trajectory_pending = false;
    m->trajectory_active = m->trajectory_duration_ms != 0U;
    if (!m->trajectory_active)
    {
        m->reference_position = m->target;
    }
    else
    {

        m->reference_velocity =
            m->trajectory_delta_position * m->trajectory_k0 / ((float)m->trajectory_duration_ms * 0.001f);
    }
}

static void RS01_UpdatePositionTrajectory(rs01_t *m, uint32_t now)
{
    quintic_plan_t plan;
    uint32_t elapsed_ms;

    if (!m->trajectory_active)
    {
        return;
    }
    elapsed_ms = now - m->trajectory_start_tick;
    if (elapsed_ms >= m->trajectory_duration_ms)
    {
        m->reference_position = m->target;
        m->reference_velocity = 0.0f;
        m->trajectory_active = false;
        return;
    }
    plan.start = m->trajectory_start_position;
    plan.delta = m->trajectory_delta_position;
    plan.k0 = m->trajectory_k0;
    plan.duration = (float)m->trajectory_duration_ms * 0.001f;
    plan.v_scale = plan.delta / plan.duration;
    Quintic_Sample(&plan, (float)elapsed_ms * 0.001f, &m->reference_position, &m->reference_velocity, NULL);
}

static HAL_StatusTypeDef tx(FDCAN_HandleTypeDef *hcan, uint8_t cmd, uint16_t extra, uint8_t id, const uint8_t *data)
{
    can_frame_t frame = {0};
    if (data != NULL)
    {
        memcpy(frame.data, data, 8U);
    }
    frame.hfdcan = hcan;
    frame.id = pack(cmd, extra, id);
    frame.id_type = FDCAN_EXTENDED_ID;
    frame.length = 8U;

    return BspCan_Send(&frame) ? HAL_OK : HAL_ERROR;
}

static bool RS01_WriteParamFloat(rs01_t *m, uint16_t index, float value)
{
    uint8_t data[8] = {0};

    memcpy(&data[0], &index, 2U);
    memcpy(&data[4], &value, 4U);

    return tx(m->hcan, RS01_CMD_WRITE_PARAM, RS01_HOST_ID, m->id, data) == HAL_OK;
}

static bool RS01_WriteParamByte(rs01_t *m, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {0};

    memcpy(&data[0], &index, 2U);
    data[4] = value;

    return tx(m->hcan, RS01_CMD_WRITE_PARAM, RS01_HOST_ID, m->id, data) == HAL_OK;
}

static bool RS01_EnsureMitMode(rs01_t *m)
{
    if (m->run_mode == RS01_RUN_MODE_MIT)
    {
        return true;
    }
    if (!RS01_WriteParamByte(m, RS01_IDX_RUN_MODE, RS01_RUN_MODE_MIT))
    {
        return false;
    }
    m->run_mode = RS01_RUN_MODE_MIT;
    return true;
}

static bool RS01_EnsureSpeedMode(rs01_t *m)
{
    if (m->run_mode == RS01_RUN_MODE_SPEED)
    {
        return true;
    }
    if (!RS01_WriteParamByte(m, RS01_IDX_RUN_MODE, RS01_RUN_MODE_SPEED))
    {
        return false;
    }
    (void)RS01_WriteParamFloat(m, RS01_IDX_LIMIT_CUR, rs01_speed_limit_current_a);
    (void)RS01_WriteParamFloat(m, RS01_IDX_ACC_RAD, rs01_speed_accel_rad_s2);
    m->run_mode = RS01_RUN_MODE_SPEED;
    return true;
}

static bool RS01_PollFeedback(rs01_t *m)
{
    uint8_t data[8];
    uint16_t zero_torque;

    if (m == NULL)
    {
        return false;
    }

    data[0] = (uint8_t)(enc(0.0f, RS01_POSITION_MIN, RS01_POSITION_MAX) >> 8);
    data[1] = (uint8_t)enc(0.0f, RS01_POSITION_MIN, RS01_POSITION_MAX);
    data[2] = (uint8_t)(enc(0.0f, -RS01_VELOCITY_MAX, RS01_VELOCITY_MAX) >> 8);
    data[3] = (uint8_t)enc(0.0f, -RS01_VELOCITY_MAX, RS01_VELOCITY_MAX);
    data[4] = (uint8_t)(enc(0.0f, 0.0f, 500.0f) >> 8);
    data[5] = (uint8_t)enc(0.0f, 0.0f, 500.0f);
    data[6] = (uint8_t)(enc(0.0f, 0.0f, 5.0f) >> 8);
    data[7] = (uint8_t)enc(0.0f, 0.0f, 5.0f);
    zero_torque = enc(0.0f, -RS01_TORQUE_MAX, RS01_TORQUE_MAX);

    return tx(m->hcan, RS01_CMD_MOTION, zero_torque, m->id, data) == HAL_OK;
}

bool RS01_BuildSpecial(rs01_t *m, uint8_t command)
{
    if (m != NULL && command == RS01_FALSE)
    {
        m->startup_hold_active = false;
    }
    return (m != NULL) && (tx(m->hcan, command, RS01_HOST_ID, m->id, NULL) == HAL_OK);
}

HAL_StatusTypeDef RS01_Init(rs01_t *m, FDCAN_HandleTypeDef *hcan, uint8_t id, int direction)
{
    if (m == NULL || hcan == NULL || id == 0U)
    {
        return HAL_ERROR;
    }
    memset(m, 0, sizeof(*m));
    m->mit_kp = RS01_MIT_POSITION_KP;
    m->mit_kd = RS01_MIT_VELOCITY_KD;
    m->hcan = hcan;
    m->id = id;
    m->direction = (direction < 0) ? -1 : 1;
    m->output_ratio = 1.0f;
    m->mode = RS_MODE_MIT;
    m->target = 0.0f;
    m->startup_hold_active = true;
    m->run_mode = 0xFFU;
    m->continuous_valid = false;
    (void)RS01_BuildSpecial(m, RS01_ENABLE);
    return HAL_OK;
}

bool RS01_Stop(rs01_t *m)
{
    bool sent;

    if (!m)
    {
        return false;
    }
    m->startup_hold_active = false;
    m->mode = RS_MODE_STOP;
    m->reference_velocity = 0.0f;
    m->trajectory_pending = false;
    m->trajectory_active = false;
    m->recovery_trajectory = false;
    sent = RS01_BuildSpecial(m, RS01_FALSE);
    m->enabled = false;
    return sent;
}

void RS01_SetTargetBasic(rs01_t *m, rs_mode_t mode, float target)
{
    if (mode == RS_MODE_MIT)
        return;

    if (m && m->target_sink)
    {
        m->target_sink(m, (int)mode, target, 0.0f);
        return;
    }

    float position_target;
    bool target_changed;

    if (!m || (mode != RS_MODE_STOP && mode != RS_MODE_POSITION && mode != RS_MODE_SPEED && mode != RS_MODE_DAMPING))
    {
        return;
    }
    if (!isfinite(target))
        return;
    if (m->mode != mode)
    {
        PID_Reset(&m->cascade_position_pid);
        PID_Reset(&m->cascade_speed_pid);
        m->cascade_tick = HAL_GetTick();
    }
    m->tau_ff_nm = 0;
    if (mode != RS_MODE_POSITION)
    {
        m->startup_hold_active = false;
        m->recovery_trajectory = false;
    }
    if (mode == RS_MODE_POSITION)
    {
        position_target = clampf(target * RS01_DEG_TO_RAD,
                                RS01_POSITION_MIN, RS01_POSITION_MAX);
        target_changed = (m->mode != RS_MODE_POSITION) || m->recovery_trajectory ||
                         (fabsf(position_target - m->target) > RS01_TRAJECTORY_EPSILON_RAD);
        m->mode = mode;
        m->target = position_target;
        m->startup_hold_active = true;
        m->recovery_trajectory = false;
        if (target_changed)
        {
            m->trajectory_pending = true;
            m->trajectory_active = false;
        }
    }
    else if (mode == RS_MODE_SPEED)
    {
        m->mode = mode;
        m->target = clampf(target, -RS01_VELOCITY_MAX, RS01_VELOCITY_MAX);
        m->trajectory_pending = false;
        m->trajectory_active = false;
        m->recovery_trajectory = false;
    }
    else
    {
        m->mode = mode;
        m->target = 0.0f;
        m->trajectory_pending = false;
        m->trajectory_active = false;
        m->recovery_trajectory = false;
    }
}

static float RS01_PlanSpeed(rs01_t *m, float target)
{
    uint32_t now = HAL_GetTick();
    float dt = m->speed_planner_valid ? (float)(uint32_t)(now - m->speed_planner_tick) * 0.001f : 0.0f;

    if (!m->speed_planner_valid || (dt > RS01_SPEED_PLANNER_RESEED_SEC))
    {
        Trapezoid_Reset(&m->speed_planner, m->velocity);
        dt = 0.0f;
    }
    else if (dt > RS01_SPEED_PLANNER_MAX_DT_SEC)
    {
        dt = RS01_SPEED_PLANNER_MAX_DT_SEC;
    }
    m->speed_planner_tick = now;
    m->speed_planner_valid = true;
    m->speed_planner.a_acc = rs01_speed_max_acceleration_rad_s2;
    m->speed_planner.a_dec = rs01_speed_max_acceleration_rad_s2;

    return Trapezoid_Update(&m->speed_planner, target, dt);
}

bool RS01_Send(rs01_t *m)
{
    uint8_t d[8];
    float pos, vel = 0, tq = 0, kp = 0, kd = 0;
    uint32_t now;
    float dt;
    if (!m)
        return false;
    if (m->mode == RS_MODE_STOP)

        return RS01_Stop(m);
    if (m->mode == RS_MODE_SPEED)
    {
        if (!RS01_EnsureSpeedMode(m))
            return false;

        return RS01_WriteParamFloat(m, RS01_IDX_SPD_REF, RS01_PlanSpeed(m, m->target));
    }
    if (!RS01_EnsureMitMode(m))
        return false;
    now = HAL_GetTick();
    dt = (now - m->cascade_tick) * 0.001f;
    m->cascade_tick = now;
    if (dt <= 0 || dt > 0.02f)
        dt = RS01_CASCADE_PERIOD_S;
    if (!m->cascade_configured)
    {
        PID_Init(&m->cascade_position_pid, RS01_POSITION_PID_KP, RS01_POSITION_PID_KI, RS01_POSITION_PID_KD,
                 RS01_POSITION_SPEED_LIMIT, RS01_POSITION_PID_I_LIMIT);
        PID_Init(&m->cascade_speed_pid, RS01_SPEED_PID_KP, RS01_SPEED_PID_KI, RS01_SPEED_PID_KD, RS01_TORQUE_MAX,
                 RS01_SPEED_PID_I_LIMIT);
        m->cascade_configured = true;
    }
    pos = (float)m->direction * (m->position + m->software_zero_offset) / m->output_ratio;
    if (m->mode == RS_MODE_POSITION)
    {
        float speed_ref;
        if (m->trajectory_pending)
            RS01_BeginPositionTrajectory(m, now);
        RS01_UpdatePositionTrajectory(m, now);
        speed_ref = PID_Calc(&m->cascade_position_pid, m->reference_position, m->position, dt);
        tq = PID_Calc(&m->cascade_speed_pid, speed_ref, m->velocity, dt) + m->tau_ff_nm;

    }
    else if (m->mode == RS_MODE_DAMPING)
    {
        kd = RS01_DAMPING_KD;
    }
    else if (m->mode == RS_MODE_MIT)
    {
        if (m->trajectory_pending)
            RS01_BeginPositionTrajectory(m, now);
        RS01_UpdatePositionTrajectory(m, now);
        pos = (float)m->direction * (m->reference_position + m->software_zero_offset) / m->output_ratio;
        vel = (float)m->direction * m->reference_velocity / m->output_ratio;
        kp = m->mit_kp;
        kd = m->mit_kd;
        tq = (float)m->direction * m->tau_ff_nm;
    }
    else
        return false;
    {
        uint16_t p = enc(pos, RS01_POSITION_MIN, RS01_POSITION_MAX);
        uint16_t v = enc(vel, -RS01_VELOCITY_MAX, RS01_VELOCITY_MAX);
        uint16_t k = enc(kp, 0, 500), damp = enc(kd, 0, 5);
        d[0] = p >> 8;
        d[1] = p;
        d[2] = v >> 8;
        d[3] = v;
        d[4] = k >> 8;
        d[5] = k;
        d[6] = damp >> 8;
        d[7] = damp;
    }

    return tx(m->hcan, RS01_CMD_MOTION, enc(tq, -RS01_TORQUE_MAX, RS01_TORQUE_MAX), m->id, d) == HAL_OK;
}

bool RS01_OnFrame(rs01_t *m, const can_frame_t *f)
{
    uint8_t cmd;
    bool recovering;
    bool had_feedback;

    if (!m || !f || f->hfdcan != m->hcan || f->id_type != FDCAN_EXTENDED_ID || f->length != 8U)
    {
        return false;
    }

    cmd = (uint8_t)((f->id >> 24) & 0x1FU);

    if (cmd != RS01_CMD_FEEDBACK || (uint8_t)f->id != RS01_HOST_ID || (uint8_t)(f->id >> 8) != m->id)
    {
        return false;
    }

    m->fault = (uint8_t)((f->id >> 16) & 0x3FU);
    m->feedback_state = (uint8_t)((f->id >> 22) & 0x03U);
    m->enabled = (m->feedback_state == 2U) && (m->fault == 0U);

    m->single_position = (float)m->direction * dec(rdbe(&f->data[0]), RS01_POSITION_MIN, RS01_POSITION_MAX) * m->output_ratio;

    m->velocity = dec(rdbe(&f->data[2]), -RS01_VELOCITY_MAX, RS01_VELOCITY_MAX) * m->output_ratio;

    m->torque = dec(rdbe(&f->data[4]), -RS01_TORQUE_MAX, RS01_TORQUE_MAX);
    m->temperature_c = (float)rdbe(&f->data[6]) * 0.1f;

    recovering = !m->feedback_valid;
    had_feedback = m->ever_feedback_received;
    if (!m->ever_feedback_received)
    {
        m->continuous_position = m->single_position;
        m->software_zero_offset = m->continuous_position;
        m->continuous_valid = true;
    }
    else if (m->continuous_valid)
    {
        m->continuous_position +=
            remainderf(m->single_position - remainderf(m->continuous_position, RS01_ANGLE_PERIOD), RS01_ANGLE_PERIOD);
    }
    m->position = m->continuous_position - m->software_zero_offset;
    m->ever_feedback_received = true;
    m->feedback_valid = true;
    m->feedback_recovery_pending = false;

    if (recovering && had_feedback)
    {
        m->hold_position = m->position;
        m->reference_position = m->position;
        m->reference_velocity = 0.0f;
        m->target = m->position;
        m->trajectory_pending = false;
        m->trajectory_active = false;
        m->startup_hold_active = false;
        m->recovery_trajectory = false;
    }

    m->offline_latched = false;
    m->feedback_recovery_pending = false;
    m->last_feedback_tick = HAL_GetTick();

    return true;
}

void RS01_ResetSoftwareZero(rs01_t *motor)
{
    if (motor == NULL || !motor->continuous_valid)
        return;
    motor->software_zero_offset = motor->continuous_position;
    motor->position = 0.0f;
    motor->hold_position = 0.0f;
    motor->reference_position = 0.0f;
    motor->target = 0.0f;
    motor->trajectory_start_position = 0.0f;
    motor->trajectory_delta_position = 0.0f;
    motor->trajectory_duration_ms = 0U;
    motor->trajectory_pending = false;
    motor->trajectory_active = false;
    PID_Reset(&motor->cascade_position_pid);
    PID_Reset(&motor->cascade_speed_pid);
}

void RS01_GetPositionDegrees(const rs01_t *m, volatile float *position_degrees)
{
    if (position_degrees != NULL)
    {
        *position_degrees = (m == NULL) ? 0.0f : m->position * 57.2957795131f;
    }
}

void RS01_Service(rs01_t *m)
{
    uint32_t now;

    if (!m)
    {
        return;
    }

    now = HAL_GetTick();

    if ((uint32_t)(now - m->last_feedback_tick) > 50U)
    {
        m->feedback_valid = false;
        m->offline_latched = true;
        m->feedback_recovery_pending = true;
    }

    if (!m->enabled && (uint32_t)(now - m->reconnect_retry_tick) >= 2U)
    {
        m->reconnect_retry_tick = now;
        (void)RS01_BuildSpecial(m, RS01_ENABLE);
    }

    if (m->mode == RS_MODE_STOP)
    {
        if ((uint32_t)(now - m->last_control_tick) >= 2U)
        {
            m->last_control_tick = now;
            if (RS01_EnsureMitMode(m))
                (void)RS01_PollFeedback(m);
        }
        return;
    }
    if (m->mode == RS_MODE_POSITION || m->mode == RS_MODE_SPEED || m->mode == RS_MODE_DAMPING || m->mode == RS_MODE_MIT)
    {
        if ((uint32_t)(now - m->last_control_tick) >= 2U)
        {
            m->last_control_tick = now;
            (void)RS01_Send(m);
        }
    }
}

void RS01_SetTargetMIT(rs01_t *motor, rs_mode_t mode, float tau_nm, float output_degrees)
{
    if (!motor || mode != RS_MODE_MIT || !isfinite(tau_nm) || !isfinite(output_degrees))
        return;
    if (motor->target_sink)
    {
        motor->target_sink(motor, (int)mode, tau_nm, output_degrees);
        return;
    }
    if (fabsf(output_degrees * RS01_DEG_TO_RAD) > RS01_POSITION_MAX || fabsf(tau_nm) > RS01_TORQUE_MAX)
        return;
    float position_target = output_degrees * RS01_DEG_TO_RAD * motor->output_ratio;
    if (motor->mode != RS_MODE_MIT || motor->recovery_trajectory ||
        fabsf(position_target - motor->target) > RS01_TRAJECTORY_EPSILON_RAD)
    {
        motor->target = position_target;
        motor->trajectory_pending = true;
        motor->trajectory_active = false;
    }
    motor->mode = RS_MODE_MIT;
    motor->tau_ff_nm = tau_nm;
    motor->startup_hold_active = true;
    PID_Reset(&motor->cascade_position_pid);
    PID_Reset(&motor->cascade_speed_pid);
}









