





#include "m3508.h"

#include <math.h>
#include "quintic_planner.h"

volatile float m3508_trajectory_max_speed_rad_s = 31.416f;
volatile float m3508_trajectory_max_acceleration_rad_s2 = 251.328f;

volatile float m3508_trajectory_time_s = 0.0f;
#include <string.h>


static int16_t m3508_command_current[M3508_MOTOR_COUNT];


static uint16_t M3508_ReadU16(const uint8_t *data                               )
{
    return ((uint16_t)data[0] << 8U) | data[1];
}


static float M3508_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}


static float M3508_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}


static void M3508_ResetMotion(m3508_t *motor)
{
    motor->reference_position_rad = motor->position_rad;
    motor->reference_speed_rad_s = 0.0f;
    motor->reference_acceleration_rad_s2 = 0.0f;
    motor->trajectory_start_rad = motor->position_rad;
    motor->trajectory_delta_rad = 0.0f;
    motor->trajectory_duration_s = 0.0f;
    motor->trajectory_elapsed_s = 0.0f;
    motor->speed_reference_valid = false;
    motor->trajectory_pending = false;
    motor->trajectory_active = false;
}


void M3508_ResetFeedback(m3508_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->feedback_online = false;
    motor->feedback_age_cycles = 0U;
    if (!motor->received)
    {
        motor->zero_stable_frames = 0U;
    }
    motor->speed_rad_s = 0.0f;
    motor->current_raw = 0;
    PID_Reset(&motor->position_pid);
    PID_Reset(&motor->speed_pid);
    PID_Reset(&motor->damping_pid);
    M3508_ResetMotion(motor);
}


static void M3508_BeginPositionTrajectory(m3508_t *motor)
{
    quintic_plan_t plan;
    float distance = motor->target_position_rad - motor->position_rad;



    Quintic_SetDefaultLimits(m3508_trajectory_max_speed_rad_s, m3508_trajectory_max_acceleration_rad_s2,
                             M3508_MIN_TRAJECTORY_S);
    Quintic_Plan(&plan, motor->position_rad, motor->target_position_rad, motor->speed_rad_s, m3508_trajectory_time_s);
    motor->trajectory_start_rad = motor->position_rad;
    motor->trajectory_delta_rad = distance;
    motor->trajectory_k0 = plan.k0;
    motor->trajectory_duration_s = plan.duration;
    motor->trajectory_elapsed_s = 0.0f;
    motor->reference_position_rad = motor->position_rad;
    motor->reference_speed_rad_s = 0.0f;
    motor->reference_acceleration_rad_s2 = 0.0f;
    motor->trajectory_pending = false;
    motor->trajectory_active = motor->trajectory_duration_s > 0.0f;
    if (motor->trajectory_active)
    {
        motor->reference_speed_rad_s =
            motor->trajectory_delta_rad * motor->trajectory_k0 / motor->trajectory_duration_s;
    }
    else
    {
        motor->reference_position_rad = motor->target_position_rad;
    }
}


static void M3508_UpdatePositionTrajectory(m3508_t *motor, float dt_s)
{
    quintic_plan_t plan;

    if (!motor->trajectory_active)
    {
        motor->reference_position_rad = motor->target_position_rad;
        motor->reference_speed_rad_s = 0.0f;
        motor->reference_acceleration_rad_s2 = 0.0f;
        return;
    }

    if (motor->trajectory_elapsed_s >= motor->trajectory_duration_s)
    {
        motor->reference_position_rad = motor->target_position_rad;
        motor->reference_speed_rad_s = 0.0f;
        motor->reference_acceleration_rad_s2 = 0.0f;
        motor->trajectory_active = false;
        return;
    }

    plan.start = motor->trajectory_start_rad;
    plan.delta = motor->trajectory_delta_rad;
    plan.k0 = motor->trajectory_k0;
    plan.duration = motor->trajectory_duration_s;
    plan.v_scale = plan.delta / plan.duration;
    Quintic_Sample(&plan, motor->trajectory_elapsed_s, &motor->reference_position_rad, &motor->reference_speed_rad_s,
                   &motor->reference_acceleration_rad_s2);
    motor->trajectory_elapsed_s += dt_s;
}


static void M3508_UpdateSpeedRamp(m3508_t *motor, float dt_s)
{
    float delta;
    float step;
    float acceleration_limit = m3508_trajectory_max_acceleration_rad_s2;

    if (!motor->speed_reference_valid)
    {
        motor->reference_speed_rad_s = motor->speed_rad_s;
        motor->speed_reference_valid = true;
    }
    delta = motor->target_speed_rad_s - motor->reference_speed_rad_s;
    step = M3508_Clamp(delta, -acceleration_limit * dt_s, acceleration_limit * dt_s);
    motor->reference_speed_rad_s += step;
    motor->reference_acceleration_rad_s2 = step / dt_s;
    motor->trajectory_active =
        M3508_Abs(motor->target_speed_rad_s - motor->reference_speed_rad_s) > M3508_TRAJECTORY_EPSILON;
}


HAL_StatusTypeDef M3508_Init(m3508_t *motor                        ,
                             FDCAN_HandleTypeDef *hcan                  ,
                             uint8_t id                                    ,
                             int direction                                  )
{
    if (motor == NULL || hcan == NULL || id == 0U || id > M3508_MOTOR_COUNT)
    {
        return HAL_ERROR;
    }
    m3508_command_current[id - 1U] = 0;
    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->id = id;
    motor->direction = (direction < 0) ? -1 : 1;
    PID_Init(&motor->position_pid, M3508_POSITION_KP, M3508_POSITION_KI, M3508_POSITION_KD, m3508_trajectory_max_speed_rad_s, M3508_POSITION_INTEGRAL_LIMIT);
    PID_Init(&motor->speed_pid, M3508_SPEED_KP, M3508_SPEED_KI, M3508_SPEED_KD, M3508_SPEED_OUTPUT_LIMIT, M3508_SPEED_INTEGRAL_LIMIT);
    PID_Init(&motor->damping_pid, 0.0f, 0.0f, motor->speed_pid.kd, motor->speed_pid.output_limit, 0.0f);
    PID_SetIntegralSeparationRange(&motor->position_pid, M3508_POSITION_I_FULL, M3508_POSITION_I_SEPARATION);
    PID_SetIntegralSeparationRange(&motor->speed_pid, M3508_SPEED_I_FULL, M3508_SPEED_I_SEPARATION);
    M3508_ResetMotion(motor);
    return HAL_OK;
}


bool M3508_OnFrame(m3508_t *motor                         ,
                   const can_frame_t *frame                      )
{
    int32_t difference;
    bool first_feedback;
    bool feedback_needs_reanchor;

    if (motor == NULL || frame == NULL || frame->hfdcan != motor->hcan || frame->id_type != FDCAN_STANDARD_ID || (frame->id != (0x200U + motor->id)) ||
        (frame->length != 8U))
    {
        return false;
    }


    first_feedback = !motor->received;
    feedback_needs_reanchor = motor->received && !motor->feedback_online;
    motor->encoder = M3508_ReadU16(&frame->data[0]);
    motor->speed_rpm = (int16_t)M3508_ReadU16(&frame->data[2]);
    motor->current_raw = (int16_t)(motor->direction * (float)(int16_t)M3508_ReadU16(&frame->data[4]));
    motor->feedback_age_cycles = 0U;

    difference = (int32_t)motor->encoder - motor->last_encoder;
    if (difference > 4096)
    {
        difference -= 8192;
    }
    else if (difference < -4096)
    {
        difference += 8192;
    }

    if (!motor->received)
    {

        motor->last_encoder = motor->encoder;
        motor->total_encoder = 0;
        motor->zero_stable_frames = 1U;
        motor->received = true;
        difference = 0;
    }
    else
    {
        motor->total_encoder += difference;
    }
    motor->last_encoder = motor->encoder;
    motor->position_rad =
        motor->direction * (float)motor->total_encoder * TWO_PI / (M3508_ENCODER_COUNT * M3508_REDUCTION_RATIO);
    motor->speed_rad_s = motor->direction * (float)motor->speed_rpm * RPM_TO_RAD_S / M3508_REDUCTION_RATIO;

    if (first_feedback)
    {

        motor->mode = M3508_MODE_POSITION;
        motor->target_position_rad = motor->position_rad;
        motor->reference_position_rad = motor->position_rad;
        motor->reference_speed_rad_s = 0.0f;
        motor->reference_acceleration_rad_s2 = 0.0f;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->startup_hold_active = true;
    }


    if (feedback_needs_reanchor && motor->received)
    {
        if (motor->mode == M3508_MODE_POSITION)
        {
            motor->target_position_rad = motor->position_rad;
        }
        else if ((motor->mode == M3508_MODE_SPEED) || (motor->mode == M3508_MODE_DAMPING))
        {
            motor->target_speed_rad_s = 0.0f;
        }
        else if (motor->mode == M3508_MODE_CURRENT)
        {
            motor->target_current_a = 0.0f;
        }
        PID_Reset(&motor->position_pid);
        PID_Reset(&motor->speed_pid);
        PID_Reset(&motor->damping_pid);
        M3508_ResetMotion(motor);
    }
    motor->feedback_online = true;
    motor->feedback_age_cycles = 0U;
    return true;
}


void M3508_SetTargetDeferred(m3508_t *motor                         ,
                     m3508_mode_t mode                          ,
                     float target                               )
{
    bool mode_changed = mode != motor->mode;
    bool target_changed;
    float target_position_rad = motor->direction * target * M3508_DEG_TO_RAD;

    if (mode != M3508_MODE_POSITION)
    {
        motor->startup_hold_active = false;
    }

    if (mode == M3508_MODE_POSITION)
    {
        target_changed =
            mode_changed || (M3508_Abs(motor->target_position_rad - target_position_rad) > M3508_TRAJECTORY_EPSILON);
    }
    else if ((mode == M3508_MODE_SPEED) || (mode == M3508_MODE_DAMPING))
    {
        target_changed = mode_changed || (M3508_Abs(motor->target_speed_rad_s - target) > M3508_TRAJECTORY_EPSILON);
    }
    else
    {
        target_changed = mode_changed || (M3508_Abs(motor->target_current_a - target) > M3508_TRAJECTORY_EPSILON);
    }

    if (mode_changed)
    {
        PID_Reset(&motor->position_pid);
        PID_Reset(&motor->speed_pid);
        PID_Reset(&motor->damping_pid);
        M3508_ResetMotion(motor);
    }

    motor->mode = mode;
    if (mode == M3508_MODE_POSITION)
    {
        motor->target_position_rad = target_position_rad;
        if (target_changed)
        {
            motor->trajectory_pending = true;
        }
        motor->speed_reference_valid = false;
    }
    else if (mode == M3508_MODE_SPEED)
    {
        motor->target_speed_rad_s = target;
        motor->trajectory_pending = false;
        if (mode_changed)
        {
            motor->speed_reference_valid = false;
        }
    }
    else if (mode == M3508_MODE_DAMPING)
    {
        motor->target_speed_rad_s = target;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->speed_reference_valid = false;
    }
    else if (mode == M3508_MODE_CURRENT)
    {
        motor->target_current_a = target;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->speed_reference_valid = false;
    }
    else
    {
        motor->target_current_a = 0.0f;
        motor->target_speed_rad_s = 0.0f;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->speed_reference_valid = false;
    }

}


void M3508_SetTarget(m3508_t *motor, m3508_mode_t mode, float target)
{
    if (motor && motor->target_sink) { motor->target_sink(motor, (int)mode, target, 0.0f); return; }

    M3508_SetTargetDeferred(motor, mode, target);
    {
        can_frame_t frame;
        uint8_t index = (uint8_t)(motor->id - 1U);
        uint8_t group_offset = (motor->id <= 4U) ? 0U : 4U;
        uint8_t first_id = (motor->id <= 4U) ? 1U : 5U;

        m3508_command_current[index] = M3508_Calc(motor, 0.002f);
        M3508_BuildGroupFrame(first_id, &m3508_command_current[group_offset], &frame);
        frame.hfdcan = motor->hcan;
        (void)BspCan_Send(&frame);
    }
}


void M3508_Service(m3508_t *motor, float dt_s)
{
    can_frame_t frame;
    uint8_t index;
    uint8_t group_offset;
    uint8_t first_id;

    if (motor == NULL)
    {
        return;
    }
    index = (uint8_t)(motor->id - 1U);
    group_offset = (motor->id <= 4U) ? 0U : 4U;
    first_id = (motor->id <= 4U) ? 1U : 5U;
    m3508_command_current[index] = M3508_Calc(motor, dt_s);
    M3508_BuildGroupFrame(first_id, &m3508_command_current[group_offset], &frame);
    frame.hfdcan = motor->hcan;
    (void)BspCan_Send(&frame);
}


bool M3508_Stop(m3508_t *motor)
{
    if (motor == NULL)
    {
        return false;
    }

    M3508_BuildSpecial(motor, M3508_DISABLE);
    M3508_SetTarget(motor, M3508_MODE_STOP, 0.0f);
    return true;
}


void M3508_SetPairPositionDegrees(m3508_t motors[2]                         ,
                                  float degrees                            )
{
    M3508_SetTarget(&motors[0], M3508_MODE_POSITION, degrees);
    M3508_SetTarget(&motors[1], M3508_MODE_POSITION, degrees);
}


int16_t M3508_Calc(m3508_t *motor                             ,
                   float dt_s                          )
{
    float target_speed;
    float current;
    float current_raw;

    if (motor->mode == M3508_MODE_STOP)
    {
        return 0;
    }

    if (motor->mode == M3508_MODE_CURRENT)
    {
        current = motor->target_current_a;
    }
    else if (motor->mode == M3508_MODE_DAMPING)
    {
        motor->damping_pid.kd = motor->speed_pid.kd;
        motor->damping_pid.output_limit = motor->speed_pid.output_limit;
        current = PID_Calc(&motor->damping_pid, motor->target_speed_rad_s, motor->speed_rad_s, dt_s);
    }
    else if (motor->mode == M3508_MODE_SPEED)
    {
        M3508_UpdateSpeedRamp(motor, dt_s);
        current = PID_Calc(&motor->speed_pid, motor->reference_speed_rad_s, motor->speed_rad_s, dt_s);
    }
    else if (motor->mode == M3508_MODE_POSITION)
    {
        float speed_limit = m3508_trajectory_max_speed_rad_s;
        if (motor->trajectory_pending)
        {
            M3508_BeginPositionTrajectory(motor);
        }
        M3508_UpdatePositionTrajectory(motor, dt_s);
        target_speed = motor->reference_speed_rad_s +
                       PID_Calc(&motor->position_pid, motor->reference_position_rad, motor->position_rad, dt_s);
        target_speed = M3508_Clamp(target_speed, -speed_limit, speed_limit);
        current = PID_Calc(&motor->speed_pid, target_speed, motor->speed_rad_s, dt_s);
    }
    else
    {
        return 0;
    }
    current = M3508_Clamp(current, -3.0f, 3.0f);
    current_raw = motor->direction * current / M3508_CURRENT_SCALE;

    if (current_raw > M3508_CURRENT_RAW_LIMIT)
    {
        current_raw = M3508_CURRENT_RAW_LIMIT;
    }
    else if (current_raw < -M3508_CURRENT_RAW_LIMIT)
    {
        current_raw = -M3508_CURRENT_RAW_LIMIT;
    }
    if (current_raw >= 0.0f)
    {
        return (int16_t)(current_raw + 0.5f);
    }
    return (int16_t)(current_raw - 0.5f);
}


void M3508_GetPositionDegrees(const m3508_t *motor, volatile float *position_degrees)
{
    if (position_degrees != NULL)
    {
        *position_degrees = (motor == NULL) ? 0.0f : motor->position_rad * 57.2957795131f;
    }
}


void M3508_BuildGroupFrame(uint8_t first_id                               ,
                           const int16_t current[4]                             ,
                           can_frame_t *frame                              )
{
    uint8_t index;

    memset(frame, 0, sizeof(*frame));
    frame->id = (first_id == 1U) ? 0x200U : 0x1FFU;
    frame->length = 8U;
    for (index = 0U; index < 4U; index++)
    {
        frame->data[index * 2U] = (uint8_t)(current[index] >> 8U);
        frame->data[index * 2U + 1U] = (uint8_t)current[index];
    }
}


void M3508_BuildSpecial(m3508_t *motor, uint8_t command)
{
    if (motor == NULL)
    {
        return;
    }
    PID_Reset(&motor->position_pid);
    PID_Reset(&motor->speed_pid);
    PID_Reset(&motor->damping_pid);
    M3508_ResetMotion(motor);
    if (command == M3508_DISABLE)
    {
        motor->startup_hold_active = false;
        motor->mode = M3508_MODE_STOP;
        motor->target_current_a = 0.0f;
        motor->target_speed_rad_s = 0.0f;
        motor->target_position_rad = motor->position_rad;
    }
}
