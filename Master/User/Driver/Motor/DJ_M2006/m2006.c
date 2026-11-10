/*
 * @file m2006.c
 * @brief 实现 M2006 电机反馈读取、串级 PID 和组帧发送。
 */

/* DJI M2006 驱动实现：处理反馈、零点建立、轨迹规划和电流控制。 */
#include "m2006.h"

#include <math.h>
#include "quintic_planner.h"

volatile float m2006_trajectory_max_speed_rad_s = 5.235987756f;
volatile float m2006_trajectory_max_acceleration_rad_s2 = 62.832f;
/* 0 = 限幅优先（按最大速度/加速度反解时长）；>0 = 时间优先（按该时长正解峰值）。 */
volatile float m2006_trajectory_time_s = 0.0f;
#include <string.h>

/* ID 1~8 的电流命令缓存：下标 0~3 对应 0x200 组，4~7 对应 0x1FF 组。 */
static int16_t m2006_command_current[M2006_MOTOR_COUNT];

/* 功能：读取 M2006 反馈帧中的大端 16 位字段；用途：解码编码器、转速和电流原始值；返回合并后的无符号整数。 */
static uint16_t M2006_ReadU16(const uint8_t *data /* M2006 反馈帧中大端 16 位字段的首地址。 */)
{
    return ((uint16_t)data[0] << 8U) | data[1];
}

/* 功能：取得浮点数绝对值；用途：计算轨迹行程和目标变化量；返回非负数值。 */
static float M2006_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 功能：将浮点数限制在指定范围；用途：约束轨迹速度和速度斜坡步长；返回限幅后的数值。 */
static float M2006_Clamp(float value, float minimum, float maximum)
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

/* 清除轨迹规划器和速度斜坡的瞬时状态。 */
static void M2006_ResetMotion(m2006_t *motor)
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

/* 清除 M2006 反馈缓存并重新开始在线计数。 */
void M2006_ResetFeedback(m2006_t *motor)
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
    M2006_ResetMotion(motor);
}

/* 根据当前位置和目标位置启动五次多项式轨迹。 */
static void M2006_BeginPositionTrajectory(m2006_t *motor)
{
    quintic_plan_t plan;
    float distance = motor->target_position_rad - motor->position_rad;

    /* 静止起动 v0=0，运行中换目标则延现实测转速。 */
    /* 规划入口：给角度和时长，算出这段运动需要的峰值速度/加速度。 */
    Quintic_SetDefaultLimits(m2006_trajectory_max_speed_rad_s, m2006_trajectory_max_acceleration_rad_s2,
                             M2006_MIN_TRAJECTORY_S);
    Quintic_Plan(&plan, motor->position_rad, motor->target_position_rad, motor->speed_rad_s, m2006_trajectory_time_s);
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

/* 按采样周期推进 M2006 位置轨迹并生成速度目标。 */
static void M2006_UpdatePositionTrajectory(m2006_t *motor, float dt_s)
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

/* 对 M2006 速度目标执行加速度限幅。 */
static void M2006_UpdateSpeedRamp(m2006_t *motor, float dt_s)
{
    float delta;
    float step;
    float acceleration_limit = m2006_trajectory_max_acceleration_rad_s2;

    if (!motor->speed_reference_valid)
    {
        motor->reference_speed_rad_s = motor->speed_rad_s;
        motor->speed_reference_valid = true;
    }
    delta = motor->target_speed_rad_s - motor->reference_speed_rad_s;
    step = M2006_Clamp(delta, -acceleration_limit * dt_s, acceleration_limit * dt_s);
    motor->reference_speed_rad_s += step;
    motor->reference_acceleration_rad_s2 = step / dt_s;
    motor->trajectory_active =
        M2006_Abs(motor->target_speed_rad_s - motor->reference_speed_rad_s) > M2006_TRAJECTORY_EPSILON;
}

/* 功能：初始化一台 M2006 的地址、方向和闭环控制状态；用途：为反馈解析与电流计算准备电机对象；无返回值表示初始化已完成。
 */
HAL_StatusTypeDef M2006_Init(m2006_t *motor /* 待初始化的 M2006 控制对象。 */,
                             FDCAN_HandleTypeDef *hcan /* FDCAN 外设句柄。 */,
                             uint8_t id /* M2006 的 DJI 协议节点编号，范围为 1 至 8。 */,
                             int direction /* 电机坐标系到机构坐标系的方向系数，取 1 或 -1。 */)
{
    if (motor == NULL || hcan == NULL || id == 0U || id > M2006_MOTOR_COUNT)
    {
        return HAL_ERROR;
    }
    m2006_command_current[id - 1U] = 0;
    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->id = id;
    motor->direction = (direction < 0) ? -1 : 1;
    PID_Init(&motor->position_pid, M2006_POSITION_KP, M2006_POSITION_KI, M2006_POSITION_KD, M2006_POSITION_OUTPUT_LIMIT, M2006_POSITION_INTEGRAL_LIMIT);
    PID_Init(&motor->speed_pid, M2006_SPEED_KP, M2006_SPEED_KI, M2006_SPEED_KD, M2006_SPEED_OUTPUT_LIMIT, M2006_SPEED_INTEGRAL_LIMIT);
    PID_SetIntegralSeparationRange(&motor->position_pid, M2006_POSITION_I_FULL, M2006_POSITION_I_SEPARATION);
    PID_SetIntegralSeparationRange(&motor->speed_pid, M2006_SPEED_I_FULL, M2006_SPEED_I_SEPARATION);
    M2006_ResetMotion(motor);
    return HAL_OK;
}

/* 功能：解析一帧 M2006 电机反馈；用途：更新转子角度、圈数、输出轴位置、速度和电流；返回 true
 * 表示该帧属于当前电机且已成功解析。 */
bool M2006_OnFrame(m2006_t *motor /* 待更新反馈的 M2006 控制对象。 */,
                   const can_frame_t *frame /* 待解析的 DJI 电机反馈帧。 */)
{
    int32_t difference; /* 当前帧与上一帧之间经过过零修正的转子编码器计数差。 */
    bool first_feedback;
    bool feedback_needs_reanchor; /* 当前反馈帧到达前是否尚未完成首次确认或经历了反馈中断。 */

    if (motor == NULL || frame == NULL || frame->hfdcan != motor->hcan || (frame->id != (0x200U + motor->id)) ||
        (frame->length != 8U))
    {
        return false;
    }

    /* 首次上电建立软件零点时保留用户已经设置的目标；只有此前已经
       收到过有效反馈、随后掉线再恢复时才需要重新锚定控制目标。 */
    first_feedback = !motor->received;
    feedback_needs_reanchor = motor->received && !motor->feedback_online;
    motor->encoder = M2006_ReadU16(&frame->data[0]);
    motor->speed_rpm = (int16_t)M2006_ReadU16(&frame->data[2]);
    motor->current_raw = (int16_t)(motor->direction * (float)(int16_t)M2006_ReadU16(&frame->data[4]));
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
        /* 第一帧有效反馈就是软件零点，避免等待期间把后续角度误当成零点。 */
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
        motor->direction * (float)motor->total_encoder * TWO_PI / (M2006_ENCODER_COUNT * M2006_REDUCTION_RATIO);
    motor->speed_rad_s = motor->direction * (float)motor->speed_rpm * RPM_TO_RAD_S / M2006_REDUCTION_RATIO;

    if (first_feedback)
    {
        /* 上电首帧把软件零点作为初始位置目标；位置命令保留该状态，切换到其他模式时才清除。 */
        motor->mode = M2006_MODE_POSITION;
        motor->target_position_rad = motor->position_rad;
        motor->reference_position_rad = motor->position_rad;
        motor->reference_speed_rad_s = 0.0f;
        motor->reference_acceleration_rad_s2 = 0.0f;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->startup_hold_active = true;
    }

    /* 恢复反馈时保留初次上电零点，仅将控制目标锚定到按原零点计算的当前位置。 */
    if (feedback_needs_reanchor && motor->received)
    {
        if (motor->mode == M2006_MODE_POSITION)
        {
            motor->target_position_rad = motor->position_rad;
        }
        else if ((motor->mode == M2006_MODE_SPEED) || (motor->mode == M2006_MODE_DAMPING))
        {
            motor->target_speed_rad_s = 0.0f;
        }
        else if (motor->mode == M2006_MODE_CURRENT)
        {
            motor->target_current_a = 0.0f;
        }
        PID_Reset(&motor->position_pid);
        PID_Reset(&motor->speed_pid);
        M2006_ResetMotion(motor);
    }
    motor->feedback_online = true;
    motor->feedback_age_cycles = 0U;
    return true;
}

/* 功能：设置 M2006
 * 的控制模式和目标值；用途：选择停止、电流、速度或位置控制并保存对应目标；无返回值表示目标已写入控制对象。 */
void M2006_SetTargetDeferred(m2006_t *motor /* 待设置目标的 M2006 控制对象。 */,
                     m2006_mode_t mode /* 本次目标采用的 M2006 控制模式。 */,
                     float target /* 电流单位 A、速度单位 rad/s、位置单位度。 */)
{
    bool mode_changed = mode != motor->mode;
    bool target_changed;
    float target_position_rad = target * M2006_DEG_TO_RAD;

    if (mode != M2006_MODE_POSITION)
    {
        motor->startup_hold_active = false;
    }

    if (mode == M2006_MODE_POSITION)
    {
        target_changed =
            mode_changed || (M2006_Abs(motor->target_position_rad - target_position_rad) > M2006_TRAJECTORY_EPSILON);
    }
    else if ((mode == M2006_MODE_SPEED) || (mode == M2006_MODE_DAMPING))
    {
        target_changed = mode_changed || (M2006_Abs(motor->target_speed_rad_s - target) > M2006_TRAJECTORY_EPSILON);
    }
    else
    {
        target_changed = mode_changed || (M2006_Abs(motor->target_current_a - target) > M2006_TRAJECTORY_EPSILON);
    }

    if (mode_changed)
    {
        PID_Reset(&motor->position_pid);
        PID_Reset(&motor->speed_pid);
        M2006_ResetMotion(motor);
    }

    motor->mode = mode;
    motor->target = target;
    if (mode == M2006_MODE_POSITION)
    {
        motor->target_position_rad = target_position_rad;
        if (target_changed)
        {
            motor->trajectory_pending = true;
        }
        motor->speed_reference_valid = false;
    }
    else if (mode == M2006_MODE_SPEED)
    {
        motor->target_speed_rad_s = target;
        motor->trajectory_pending = false;
        if (mode_changed)
        {
            motor->speed_reference_valid = false;
        }
    }
    else if (mode == M2006_MODE_DAMPING)
    {
        motor->target_speed_rad_s = target;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->speed_reference_valid = false;
    }
    else if (mode == M2006_MODE_CURRENT)
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

void M2006_SetTarget(m2006_t *motor, m2006_mode_t mode, float target)
{
    if (motor && motor->target_sink) { motor->target_sink(motor, (int)mode, target, 0.0f); return; }

    M2006_SetTargetDeferred(motor, mode, target);
    {
        can_frame_t frame;
        uint8_t index = (uint8_t)(motor->id - 1U);
        uint8_t group_offset = (motor->id <= 4U) ? 0U : 4U;
        uint8_t first_id = (motor->id <= 4U) ? 1U : 5U;

        m2006_command_current[index] = M2006_Calc(motor, M2006_TARGET_UPDATE_PERIOD_S);
        M2006_BuildGroupFrame(first_id, &m2006_command_current[group_offset], &frame);
        frame.hfdcan = motor->hcan;
        (void)BspCan_Send(&frame);
    }
}

/* 周期维护 M2006 软件零点、反馈在线状态和控制输出。 */
void M2006_Service(m2006_t *motor, float dt_s)
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
    m2006_command_current[index] = M2006_Calc(motor, dt_s);
    M2006_BuildGroupFrame(first_id, &m2006_command_current[group_offset], &frame);
    frame.hfdcan = motor->hcan;
    (void)BspCan_Send(&frame);
}

/* 停止 M2006 并清除位置/速度轨迹状态。 */
bool M2006_Stop(m2006_t *motor)
{
    if (motor == NULL)
    {
        return false;
    }

    M2006_BuildSpecial(motor, M2006_DISABLE);
    M2006_SetTarget(motor, M2006_MODE_STOP, 0.0f);
    return true;
}

/* 功能：设置 M2006 输出轴角度目标；用途：在 M2006 电机库内完成角度换算并启动位置轨迹；无返回值表示目标已写入。 */
void M2006_SetPositionDegrees(m2006_t *motor /* 待控制的 M2006 对象。 */,
                              float degrees /* M2006 输出轴目标角度，单位：度。 */)
{
    M2006_SetTarget(motor, M2006_MODE_POSITION, degrees);
}

/* 功能：计算 M2006 本周期需要发送的电流指令；用途：按当前模式执行速度环或位置速度串级控制；返回值表示经过限幅的 DJI
 * 协议电流原始值。 */
int16_t M2006_Calc(m2006_t *motor /* 参与本周期闭环计算的 M2006 控制对象。 */,
                   float dt_s /* 相邻两次控制计算的时间间隔，单位：秒。 */)
{
    float target_speed; /* 速度内环本周期使用的 M2006 输出轴目标速度，单位：弧度每秒。 */
    float current;      /* 控制器本周期计算得到的 M2006 目标电流，单位：安培。 */
    float current_raw;  /* 目标电流换算得到的 M2006 协议原始值。 */

    if (motor->mode == M2006_MODE_STOP)
    {
        return 0;
    }

    if (motor->mode == M2006_MODE_CURRENT)
    {
        current = motor->target_current_a;
    }
    else if (motor->mode == M2006_MODE_DAMPING)
    {
        current = M2006_Clamp(M2006_DAMPING_GAIN_A_PER_RAD_S * (motor->target_speed_rad_s - motor->speed_rad_s),
                              -M2006_DAMPING_CURRENT_LIMIT_A, M2006_DAMPING_CURRENT_LIMIT_A);
    }
    else if (motor->mode == M2006_MODE_SPEED)
    {
        M2006_UpdateSpeedRamp(motor, dt_s);
        current = PID_Calc(&motor->speed_pid, motor->reference_speed_rad_s, motor->speed_rad_s, dt_s);
    }
    else if (motor->mode == M2006_MODE_POSITION)
    {
        float speed_limit = m2006_trajectory_max_speed_rad_s;
        if (motor->trajectory_pending)
        {
            M2006_BeginPositionTrajectory(motor);
        }
        M2006_UpdatePositionTrajectory(motor, dt_s);
        target_speed = motor->reference_speed_rad_s +
                       PID_Calc(&motor->position_pid, motor->reference_position_rad, motor->position_rad, dt_s);
        target_speed = M2006_Clamp(target_speed, -speed_limit, speed_limit);
        current = PID_Calc(&motor->speed_pid, target_speed, motor->speed_rad_s, dt_s);
    }
    else
    {
        return 0;
    }
    current_raw = motor->direction * current / M2006_CURRENT_SCALE;

    if (current_raw > M2006_CURRENT_RAW_LIMIT)
    {
        current_raw = M2006_CURRENT_RAW_LIMIT;
    }
    else if (current_raw < -M2006_CURRENT_RAW_LIMIT)
    {
        current_raw = -M2006_CURRENT_RAW_LIMIT;
    }
    if (current_raw >= 0.0f)
    {
        return (int16_t)(current_raw + 0.5f);
    }
    return (int16_t)(current_raw - 0.5f);
}

/* 读取 M2006 输出轴位置并转换为角度。 */
void M2006_GetPositionDegrees(const m2006_t *motor, volatile float *position_degrees)
{
    if (position_degrees != NULL)
    {
        *position_degrees = (motor == NULL) ? 0.0f : motor->position_rad * 57.2957795131f;
    }
}

/* 功能：把四台连续编号 M2006 的电流指令组成 DJI 分组控制帧；用途：通过一帧 CAN
 * 数据同时控制四台电机；无返回值表示帧标识符和数据段已写入。 */
void M2006_BuildGroupFrame(uint8_t first_id /* 本帧第一个电流通道对应的 M2006 节点编号。 */,
                           const int16_t current[4] /* 四台连续编号 M2006 的协议电流原始值。 */,
                           can_frame_t *frame /* 用于写出 DJI 分组电流命令的 CAN 帧。 */)
{
    uint8_t index; /* 当前写入分组控制帧的 M2006 电流通道下标。 */

    memset(frame, 0, sizeof(*frame));
    frame->id = (first_id == 1U) ? 0x200U : 0x1FFU;
    frame->length = 8U;
    for (index = 0U; index < 4U; index++)
    {
        frame->data[index * 2U] = (uint8_t)(current[index] >> 8U);
        frame->data[index * 2U + 1U] = (uint8_t)current[index];
    }
}

/* 构造 M2006 特殊控制命令并写入待发送帧。 */
void M2006_BuildSpecial(m2006_t *motor, uint8_t command)
{
    if (motor == NULL)
    {
        return;
    }
    PID_Reset(&motor->position_pid);
    PID_Reset(&motor->speed_pid);
    M2006_ResetMotion(motor);
    if (command == M2006_DISABLE)
    {
        motor->startup_hold_active = false;
        motor->mode = M2006_MODE_STOP;
        motor->target_current_a = 0.0f;
        motor->target_speed_rad_s = 0.0f;
        motor->target_position_rad = motor->position_rad;
    }
}
