/*
 * @file j4310.c
 * @brief 实现 J4310 反馈解析和控制指令编码。
 */

/* DM J4310 驱动实现：完成 MIT 协议打包、反馈解码和关节轨迹控制。 */
#include "j4310.h"

#include "quintic_planner.h"
#include "trapezoid_planner.h"
#include <math.h>

volatile float j4310_trajectory_max_speed_rad_s = 3.7f;
volatile float j4310_trajectory_max_acceleration_rad_s2 = 15.0f;
/* 0 = 限幅优先（按最大速度/加速度反解时长）；>0 = 时间优先（按该时长正解峰值）。 */
volatile float j4310_trajectory_time_s = 0.0f;
/* 速度模式加速度上限：默认与五次轨迹同值，按负载惯量在 变量观察窗口 里调。 */
volatile float j4310_speed_max_acceleration_rad_s2 = 15.0f;
/* MIT 转矩前馈占位：先给 0，标定后直接改。 */
volatile float j4310_tau_ff_nm = 0.0f;

#include <string.h>

extern volatile uint8_t g_motor_power_recovery_enable;

static j4310_online_mit_state_t j4310_online_mit;
static uint32_t j4310_feedback_sequence;
static uint32_t j4310_online_feedback_sequence;
/* J4310_SetTargetRadians：执行该接口对应的控制或数据处理流程。 */
static void J4310_SetTargetRadians(j4310_t *motor, float position_rad, float speed_rad_s, float kp, float kd,
                                   float torque_nm);

/* 将 J4310 在线控制参数限制在协议上下界。 */
static float J4310_OnlineClamp(float value, float minimum, float maximum)
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

/* 平滑逼近目标，降低 J4310 上电和恢复过程的冲击。 */
static float J4310_OnlineApproach(float current, float target, float smoothing)
{
    return current + (target - current) * smoothing;
}

/* 判断 J4310 反馈状态是否表示故障。 */
static bool J4310_IsFaultState(uint8_t state)
{
    return (state == 0x02U) || (state == 0x03U) || (state == 0x05U) || (state == 0x07U) ||
           ((state >= 0x08U) && (state <= 0x0EU));
}

/* 将位置误差折算到最短角度路径。 */
static float J4310_WrapPositionDelta(float delta_rad)
{
    while (delta_rad > J4310_PI)
    {
        delta_rad -= J4310_TWO_PI;
    }
    while (delta_rad < -J4310_PI)
    {
        delta_rad += J4310_TWO_PI;
    }
    return delta_rad;
}

/* J4310_OnlineMitUpdate：执行该接口对应的控制或数据处理流程。 */
static void J4310_OnlineMitUpdate(const j4310_t *motor, float position_rad, float speed_rad_s, float kp, float kd,
                                  float *applied_kp, float *applied_kd)
{
    float position_error;
    float absolute_error;
    float velocity_error;
    float error_ratio;
    float velocity_ratio;
    float kp_scale;
    float kd_scale;
    float target_kp;
    float target_kd;

    if ((applied_kp == 0) || (applied_kd == 0))
    {
        return;
    }
    if ((kp != j4310_online_mit.base_kp) || (kd != j4310_online_mit.base_kd))
    {
        j4310_online_mit.base_kp = J4310_OnlineClamp(kp, 0.0f, 49.0f);
        j4310_online_mit.base_kd = J4310_OnlineClamp(kd, 0.0f, 0.95f);
        j4310_online_mit.applied_kp = j4310_online_mit.base_kp;
        j4310_online_mit.applied_kd = j4310_online_mit.base_kd;
        j4310_online_mit.previous_error = 0.0f;
        j4310_online_mit.previous_abs_error = 0.0f;
        j4310_online_mit.convergence_rate = 0.0f;
        j4310_online_mit.started = false;
    }
    if (!j4310_online_mit.enabled || (j4310_online_feedback_sequence == j4310_feedback_sequence) ||
        !motor->feedback.received)
    {
        *applied_kp = j4310_online_mit.applied_kp;
        *applied_kd = j4310_online_mit.applied_kd;
        return;
    }

    j4310_online_feedback_sequence = j4310_feedback_sequence;
    position_error = position_rad - motor->feedback.position_rad;
    absolute_error = fabsf(position_error);
    velocity_error = speed_rad_s - motor->feedback.speed_rad_s;
    if ((absolute_error <= 0.01745329252f) && (fabsf(motor->feedback.speed_rad_s) <= 0.10f))
    {
        j4310_online_mit.applied_kp = j4310_online_mit.base_kp;
        j4310_online_mit.applied_kd = j4310_online_mit.base_kd;
        j4310_online_mit.previous_error = position_error;
        j4310_online_mit.previous_abs_error = absolute_error;
        j4310_online_mit.convergence_rate = 0.0f;
        j4310_online_mit.started = true;
        *applied_kp = j4310_online_mit.applied_kp;
        *applied_kd = j4310_online_mit.applied_kd;
        return;
    }

    error_ratio = J4310_OnlineClamp((absolute_error - 0.01745329252f) / (0.17453292520f - 0.01745329252f), 0.0f, 1.0f);
    velocity_ratio = J4310_OnlineClamp(fabsf(velocity_error) / 3.0f, 0.0f, 1.0f);
    kp_scale = 1.0f + 0.30f * error_ratio;
    kd_scale = 1.0f + 0.20f * (1.0f - error_ratio) + 0.25f * velocity_ratio;
    if (j4310_online_mit.started)
    {
        float instantaneous_rate;

        instantaneous_rate = (j4310_online_mit.previous_abs_error - absolute_error) / 0.001f;
        j4310_online_mit.convergence_rate = 0.8f * j4310_online_mit.convergence_rate + 0.2f * instantaneous_rate;
        if ((position_error * j4310_online_mit.previous_error) < 0.0f)
        {
            kp_scale *= 0.75f;
            kd_scale *= 1.35f;
        }
        else if (j4310_online_mit.convergence_rate < -0.05f)
        {
            kp_scale *= 0.85f;
            kd_scale *= 1.25f;
        }
        else if ((absolute_error > 0.01745329252f) && (fabsf(j4310_online_mit.convergence_rate) < 0.01f) &&
                 (fabsf(motor->feedback.speed_rad_s) < 0.10f))
        {
            kp_scale *= 1.12f;
        }
    }
    if ((absolute_error <= 0.01745329252f) && (fabsf(motor->feedback.speed_rad_s) > 0.10f))
    {
        kp_scale *= 0.85f;
        kd_scale *= 1.15f;
    }
    kp_scale = J4310_OnlineClamp(kp_scale, 0.65f, 1.40f);
    kd_scale = J4310_OnlineClamp(kd_scale, 0.80f, 1.80f);
    target_kp = J4310_OnlineClamp(j4310_online_mit.base_kp * kp_scale, 0.0f, 49.0f);
    target_kd = J4310_OnlineClamp(j4310_online_mit.base_kd * kd_scale, 0.0f, 0.95f);
    j4310_online_mit.applied_kp = J4310_OnlineApproach(j4310_online_mit.applied_kp, target_kp, 0.20f);
    j4310_online_mit.applied_kd = J4310_OnlineApproach(j4310_online_mit.applied_kd, target_kd, 0.20f);
    j4310_online_mit.previous_error = position_error;
    j4310_online_mit.previous_abs_error = absolute_error;
    j4310_online_mit.started = true;
    *applied_kp = j4310_online_mit.applied_kp;
    *applied_kd = j4310_online_mit.applied_kd;
}

/* 功能：根据节点编号和控制模式生成 J4310 控制 CAN 标识符；用于区分 MIT 模式与位置速度模式的协议地址。 */
static uint16_t J4310_GetControlId(const j4310_t *motor /* 提供节点编号和控制模式的 J4310 控制对象。 */)
{
    if (motor->mode == J4310_MODE_SPEED)
    {
        return motor->motor_id + 0x200U;
    }
    return motor->motor_id;
}

/* 功能：把 J4310
 * 协议无符号整数还原为物理量；用途：解码反馈帧中的位置、速度和转矩字段；返回值表示按指定范围换算后的物理量。 */
static float J4310_UintToFloat(uint16_t value /* 待解码的 J4310 协议无符号整数。 */,
                               float min /* 解码结果所代表物理量的最小值。 */,
                               float max /* 解码结果所代表物理量的最大值。 */,
                               uint8_t bits /* 待解码协议字段的有效位数。 */)
{
    return (float)value * (max - min) / (float)((1UL << bits) - 1UL) + min;
}

/* 功能：把物理量按指定范围量化为 J4310 协议整数；用途：编码 MIT
 * 控制帧的位置、速度、增益和转矩字段；返回值表示限幅并量化后的无符号整数。 */
static uint16_t J4310_FloatToUint(float value /* 待编码到 J4310 协议字段的物理量。 */,
                                  float min /* 编码前允许输入的物理量最小值。 */,
                                  float max /* 编码前允许输入的物理量最大值。 */,
                                  uint8_t bits /* 编码结果需要占用的有效位数。 */)
{
    if (value < min)
    {
        value = min;
    }
    else if (value > max)
    {
        value = max;
    }
    return (uint16_t)((value - min) * (float)((1UL << bits) - 1UL) / (max - min));
}

/* 功能：构造 J4310 的特殊控制命令帧；用途：生成使能或失能所需的固定数据格式；无返回值表示 CAN 帧已写入。 */
static void J4310_BuildSpecial(const j4310_t *motor /* 用于选择特殊命令 CAN 标识符的 J4310 控制对象。 */,
                               uint8_t command /* 待写入数据段末字节的 J4310 特殊指令码。 */,
                               can_frame_t *frame /* 用于写出 J4310 特殊命令的 CAN 帧。 */)
{
    memset(frame, 0xFF, sizeof(*frame));
    frame->hfdcan = NULL;
    frame->id = J4310_GetControlId(motor);
    frame->length = 8U;
    frame->data[7] = command;
}

/* 轨迹时长改由通用五次规划器 Quintic_Plan 计算，原解析公式已移除。 */

/* 功能：以实测位置为起点建立 J4310 五次位置轨迹；用途：将新关节目标转换为单电机平滑运动；无返回值表示轨迹已建立。 */
static void J4310_BeginPositionTrajectory(j4310_t *motor, uint32_t now)
{
    quintic_plan_t plan;
    float duration_ms;
    float speed_limit = j4310_trajectory_max_speed_rad_s;
    float acceleration_limit = j4310_trajectory_max_acceleration_rad_s2;
    float v0 = motor->feedback.received ? motor->feedback.speed_rad_s : 0.0f;

    motor->trajectory_start_rad =
        motor->feedback.received ? motor->feedback.position_rad : motor->reference_position_rad;
    /* 自动回零沿用原来的 1/3 限幅。 */
    if (motor->recovery_trajectory)
    {
        speed_limit *= 0.3333333333f;
        acceleration_limit *= 0.3333333333f;
    }
    /* 静止起动 v0=0，运行中换目标则延现实测关节速度。 */
    /* 规划入口：给角度和时长，算出这段运动需要的峰值速度/加速度。 */
    Quintic_SetDefaultLimits(speed_limit, acceleration_limit, (float)J4310_TRAJECTORY_PERIOD_MS * 0.001f);
    Quintic_Plan(&plan, motor->trajectory_start_rad, motor->target_position_rad, v0, j4310_trajectory_time_s);
    motor->trajectory_k0 = plan.k0;
    motor->trajectory_started_at = now;
    duration_ms = plan.duration * 1000.0f;
    if (duration_ms > 0.0f)
    {
        duration_ms = ceilf(duration_ms / (float)J4310_TRAJECTORY_PERIOD_MS) * (float)J4310_TRAJECTORY_PERIOD_MS;
    }
    motor->trajectory_duration_ms = (uint32_t)duration_ms;
    motor->reference_position_rad = motor->trajectory_start_rad;
    motor->reference_speed_rad_s = 0.0f;
    motor->trajectory_pending = false;
    motor->trajectory_active = motor->trajectory_duration_ms != 0U;
    if (!motor->trajectory_active)
    {
        motor->reference_position_rad = motor->target_position_rad;
        motor->reference_speed_rad_s = motor->target_speed_rad_s;
    }
    else
    {
        motor->reference_speed_rad_s = (motor->target_position_rad - motor->trajectory_start_rad) *
                                       motor->trajectory_k0 / ((float)motor->trajectory_duration_ms * 0.001f);
    }
}

/* 功能：按当前时刻采样 J4310 五次位置轨迹；用途：生成与 B
 * 组一致且满足速度和加速度限制的平滑给定；无返回值表示本周期参考状态已更新。 */
static void J4310_SamplePositionTrajectory(j4310_t *motor, uint32_t now)
{
    uint32_t elapsed_ms;
    quintic_plan_t plan;

    if (!motor->trajectory_active)
    {
        return;
    }
    elapsed_ms = now - motor->trajectory_started_at;
    if (elapsed_ms >= motor->trajectory_duration_ms)
    {
        motor->reference_position_rad = motor->target_position_rad;
        motor->reference_speed_rad_s = motor->target_speed_rad_s;
        motor->trajectory_active = false;
        return;
    }

    plan.start = motor->trajectory_start_rad;
    plan.delta = motor->target_position_rad - motor->trajectory_start_rad;
    plan.k0 = motor->trajectory_k0;
    plan.duration = (float)motor->trajectory_duration_ms * 0.001f;
    plan.v_scale = plan.delta / plan.duration;
    Quintic_Sample(&plan, (float)elapsed_ms * 0.001f, &motor->reference_position_rad, &motor->reference_speed_rad_s,
                   NULL);
}

/* 功能：初始化 J4310 的电机地址、主控地址和控制模式；用途：为反馈解析和命令组帧建立协议配置；无返回值表示初始化已完成。
 */
void J4310_Init(j4310_t *motor /* 待初始化的 J4310 控制对象。 */,
                uint8_t motor_id /* J4310 在电机侧使用的协议节点编号。 */,
                uint16_t master_id /* J4310 反馈帧发送到的主控节点编号。 */,
                int direction /* 电机坐标系到机构坐标系的方向系数，取 1 或 -1。 */,
                j4310_mode_t mode /* J4310 初始化后采用的控制模式。 */)
{
    memset(motor, 0, sizeof(*motor));
    motor->motor_id = motor_id;
    motor->master_id = master_id;
    motor->direction = (direction < 0) ? -1 : 1;
    motor->mode = mode;
    motor->startup_hold_pending = true;
    memset(&j4310_online_mit, 0, sizeof(j4310_online_mit));
    j4310_online_mit.enabled = true;
    j4310_feedback_sequence = 0U;
    j4310_online_feedback_sequence = 0U;
}

/* 功能：切换 J4310 当前使用的控制模式；用途：决定后续控制帧采用 MIT
 * 或位置速度协议格式；无返回值表示模式已写入控制对象。 */
void J4310_SetMode(j4310_t *motor /* 待切换模式的 J4310 控制对象。 */, j4310_mode_t mode /* J4310 新的控制模式。 */)
{
    motor->mode = mode;
}

void J4310_SetTargetMode(j4310_t *motor, j4310_mode_t mode, float target, float speed_rad_s, float kp, float kd,
                         float torque_nm)
{
    (void)speed_rad_s;
    (void)kp;
    (void)kd;
    if (mode == J4310_MODE_MIT)
        J4310_SetTargetMIT(motor, mode, torque_nm, target);
    else
        J4310_SetTargetBasic(motor, mode, target);
}

/* 停止 J4310 并保持当前位置，清除运动轨迹。 */
bool J4310_Stop(j4310_t *motor)
{
    if (motor == NULL)
    {
        return false;
    }

    motor->mode = J4310_MODE_STOP;
    motor->target_position_rad =
        motor->feedback.received ? motor->feedback.position_rad : motor->reference_position_rad;
    motor->reference_position_rad = motor->target_position_rad;
    motor->target_speed_rad_s = 0.0f;
    motor->reference_speed_rad_s = 0.0f;
    motor->kp = 0.0f;
    motor->kd = 0.0f;
    motor->target_torque_nm = 0.0f;
    motor->trajectory_pending = false;
    motor->trajectory_active = false;
    motor->recovery_trajectory = false;
    return true;
}

/* 读取 J4310 关节位置并转换为角度。 */
void J4310_GetPositionDegrees(const j4310_t *motor, volatile float *position_degrees)
{
    if (position_degrees != NULL)
    {
        *position_degrees = (motor == NULL) ? 0.0f : motor->feedback.position_rad * 57.2957795131f;
    }
}

/* 请求 J4310 进入使能状态并维护重试计数。 */
void J4310_RequestEnable(j4310_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->startup_hold_pending = true;
    motor->startup_enable_countdown = 0U;
    motor->feedback.received = false;
    motor->feedback.age_cycles = 0U;
    j4310_online_mit.applied_kp = j4310_online_mit.base_kp;
    j4310_online_mit.applied_kd = j4310_online_mit.base_kd;
    j4310_online_mit.previous_error = 0.0f;
    j4310_online_mit.previous_abs_error = 0.0f;
    j4310_online_mit.convergence_rate = 0.0f;
    j4310_online_mit.started = false;
    j4310_online_feedback_sequence = j4310_feedback_sequence;
}

/* 保存使用弧度表示的 J4310 内部目标。 */
static void J4310_SetTargetRadians(j4310_t *motor, float position_rad, float speed_rad_s, float kp, float kd,
                                   float torque_nm)
{
    bool position_changed = fabsf(motor->target_position_rad - position_rad) > J4310_TRAJECTORY_EPSILON_RAD;
    bool position_control_started = (motor->kp <= 0.0f) && (kp > 0.0f);

    motor->target_position_rad = position_rad;
    motor->target_speed_rad_s = speed_rad_s;
    motor->kp = kp;
    motor->kd = kd;
    motor->target_torque_nm = torque_nm;
    motor->startup_hold_active = (kp > 0.0f);
    motor->recovery_trajectory = false;
    if (position_control_started)
    {
        J4310_RequestEnable(motor);
    }
    if (kp <= 0.0f)
    {
        motor->reference_position_rad = position_rad;
        motor->reference_speed_rad_s = speed_rad_s;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
    }
    else if (position_changed || position_control_started)
    {
        motor->trajectory_pending = true;
    }
}

/* 功能：设置 J4310 的位置、速度、增益和前馈转矩目标；位置输入使用度，内部自动转换为弧度。 */
void J4310_SetMitTarget(j4310_t *motor /* 待设置 MIT 目标的 J4310 控制对象。 */,
                        float position_degrees /* J4310 的目标关节位置，单位：度。 */,
                        float speed_rad_s /* J4310 的目标关节速度，单位：弧度每秒。 */,
                        float kp /* J4310 MIT 位置误差的比例增益。 */, float kd /* J4310 MIT 速度微分增益。 */,
                        float torque_nm /* J4310 需要叠加的前馈转矩，单位：牛米。 */)
{
    if (motor == NULL)
    {
        return;
    }

    J4310_SetTargetRadians(motor, position_degrees * J4310_DEG_TO_RAD, speed_rad_s, kp, kd, torque_nm);
}

/* 功能：以度为单位设置 J4310 关节位置目标；用途：在 J4310 电机库内完成角度换算并启动平滑轨迹；无返回值表示目标已写入。
 */
void J4310_SetPositionDegrees(j4310_t *motor, float degrees, float kp, float kd, float torque_nm)
{
    (void)kp;
    (void)kd;
    (void)torque_nm; /* Legacy signature; POSITION is always cascaded. */
    J4310_SetTargetBasic(motor, J4310_MODE_POSITION, degrees);
}

/* 功能：解析一帧 J4310 电机反馈；用途：更新关节位置、速度、转矩和状态信息；返回 true 表示该帧属于当前电机且已成功解析。
 */
bool J4310_OnFrame(j4310_t *motor /* 待更新反馈的 J4310 控制对象。 */,
                   const can_frame_t *frame /* 待解析的 J4310 反馈帧。 */)
{
    uint16_t position;        /* 从 J4310 反馈帧提取的关节位置协议原始值。 */
    uint16_t speed;           /* 从 J4310 反馈帧提取的关节速度协议原始值。 */
    uint16_t torque;          /* 从 J4310 反馈帧提取的输出转矩协议原始值。 */
    float raw_position_rad;   /* 按机构方向换算、尚未连续展开的协议位置。 */
    float position_delta_rad; /* 相邻两帧协议位置经过回绕修正后的变化量。 */
    bool was_fault_active;    /* 解析本帧之前是否处于故障失能状态。 */
    bool first_feedback;

    if ((frame->id != motor->master_id) || (frame->length != 8U) || ((frame->data[0] & 0x0FU) != motor->motor_id))
    {
        return false;
    }

    position = ((uint16_t)frame->data[1] << 8U) | frame->data[2];
    speed = ((uint16_t)frame->data[3] << 4U) | (frame->data[4] >> 4U);
    torque = ((uint16_t)(frame->data[4] & 0x0FU) << 8U) | frame->data[5];
    was_fault_active = motor->fault_active;
    first_feedback = !motor->position_tracking_valid;
    motor->feedback.state = frame->data[0] >> 4U;
    raw_position_rad = motor->direction * J4310_UintToFloat(position, -J4310_POSITION_MAX, J4310_POSITION_MAX, 16U);
    if (!motor->position_tracking_valid)
    {
        motor->feedback.position_rad = raw_position_rad;
        motor->position_tracking_valid = true;
    }
    else
    {
        position_delta_rad = raw_position_rad - motor->last_raw_position_rad;
        motor->feedback.position_rad += J4310_WrapPositionDelta(position_delta_rad);
    }
    motor->last_raw_position_rad = raw_position_rad;
    motor->position_offset_rad = raw_position_rad - motor->feedback.position_rad;
    motor->feedback.speed_rad_s = motor->direction * J4310_UintToFloat(speed, -J4310_SPEED_MAX, J4310_SPEED_MAX, 12U);
    motor->feedback.torque_nm = motor->direction * J4310_UintToFloat(torque, -J4310_TORQUE_MAX, J4310_TORQUE_MAX, 12U);
    motor->feedback.mos_temperature = frame->data[6];
    motor->feedback.rotor_temperature = frame->data[7];
    motor->feedback.received = true;
    motor->feedback.age_cycles = 0U;
    motor->offline_latched = false;
    j4310_feedback_sequence++;

    if (J4310_IsFaultState(motor->feedback.state))
    {
        if (!was_fault_active)
        {
            motor->fault_disable_sent = false;
        }
        motor->fault_active = true;
        motor->fault_recovery_pending = false;
        return true;
    }
    if (was_fault_active)
    {
        motor->fault_active = false;
        motor->fault_disable_sent = false;
        motor->fault_recovery_pending = true;
        motor->startup_enable_countdown = 0U;
        return true;
    }
    if (motor->fault_recovery_pending)
    {
        if (motor->feedback.state == J4310_STATE_ENABLED)
        {
            motor->fault_recovery_pending = false;
        }
        else
        {
            return true;
        }
    }
    if ((motor->feedback.state == 0U) && !motor->startup_hold_pending)
    {
        J4310_RequestEnable(motor);
    }
    if (motor->startup_hold_pending && (motor->feedback.state == J4310_STATE_ENABLED))
    {
        motor->reference_position_rad = motor->feedback.position_rad;
        motor->reference_speed_rad_s = 0.0f;
        motor->trajectory_start_rad = motor->feedback.position_rad;
        motor->trajectory_active = false;
        if (!motor->trajectory_pending)
        {
            motor->target_position_rad = motor->feedback.position_rad;
            motor->target_speed_rad_s = 0.0f;
            /* 首帧确认后只维持通信，不施加上电位置保持转矩。 */
            motor->kp = 0.0f;
            motor->kd = 0.0f;
            motor->target_torque_nm = 0.0f;
            motor->startup_hold_active = false;
        }
        if (first_feedback && g_motor_power_recovery_enable == 1U)
        {
            motor->mode = J4310_MODE_MIT;
            motor->target_position_rad = 0.0f;
            motor->target_speed_rad_s = 0.0f;
            motor->reference_position_rad = motor->feedback.position_rad;
            motor->trajectory_start_rad = motor->feedback.position_rad;
            motor->trajectory_pending = true;
            motor->trajectory_active = false;
            motor->startup_hold_active = true;
            motor->recovery_trajectory = true;
            if (motor->kp <= 0.0f)
            {
                motor->kp = J4310_STARTUP_HOLD_KP;
            }
            if (motor->kd <= 0.0f)
            {
                motor->kd = J4310_STARTUP_HOLD_KD;
            }
        }
        motor->startup_hold_pending = false;
    }
    return true;
}

/* 功能：构造 J4310 电机使能帧；用途：请求电机进入可接收控制指令的工作状态；无返回值表示使能帧已写入。 */
void J4310_BuildEnable(const j4310_t *motor /* 提供使能命令目标地址的 J4310 控制对象。 */,
                       can_frame_t *frame /* 用于写出 J4310 使能命令的 CAN 帧。 */)
{
    J4310_BuildSpecial(motor, J4310_ENABLE_COMMAND, frame);
}

/* 功能：构造 J4310 电机失能帧；用途：请求电机停止力矩输出并退出工作状态；无返回值表示失能帧已写入。 */
void J4310_BuildDisable(const j4310_t *motor /* 提供失能命令目标地址的 J4310 控制对象。 */,
                        can_frame_t *frame /* 用于写出 J4310 失能命令的 CAN 帧。 */)
{
    J4310_BuildSpecial(motor, J4310_DISABLE_COMMAND, frame);
}

/* 功能：构造 J4310 的 MIT
 * 控制帧；用途：把位置、速度、比例增益、微分增益和转矩目标编码后下发；无返回值表示控制帧已写入。 */
void J4310_BuildMit(const j4310_t *motor /* 提供协议地址和模式的 J4310 控制对象。 */,
                    float position_rad /* 待编码的 J4310 目标关节位置，单位：弧度。 */,
                    float speed_rad_s /* 待编码的 J4310 目标关节速度，单位：弧度每秒。 */,
                    float kp /* 待编码的 J4310 MIT 比例增益。 */, float kd /* 待编码的 J4310 MIT 微分增益。 */,
                    float torque_nm /* 待编码的 J4310 前馈转矩，单位：牛米。 */,
                    can_frame_t *frame /* 用于写出 J4310 MIT 命令的 CAN 帧。 */)
{
    float applied_kp;
    float applied_kd;
    float protocol_position_rad = position_rad;
    uint16_t position;
    uint16_t speed;
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t torque;

    if (motor->position_tracking_valid)
    {
        protocol_position_rad += motor->position_offset_rad;
    }
    position = J4310_FloatToUint(motor->direction * protocol_position_rad, -J4310_POSITION_MAX, J4310_POSITION_MAX,
                                 16U); /* J4310 目标关节位置编码后的 16 位协议值。 */
    speed = J4310_FloatToUint(motor->direction * speed_rad_s, -J4310_SPEED_MAX, J4310_SPEED_MAX,
                              12U);                          /* J4310 目标关节速度编码后的 12 位协议值。 */
    kp_raw = J4310_FloatToUint(kp, 0.0f, J4310_KP_MAX, 12U); /* J4310 MIT 比例增益编码后的 12 位协议值。 */
    kd_raw = J4310_FloatToUint(kd, 0.0f, J4310_KD_MAX, 12U); /* J4310 MIT 微分增益编码后的 12 位协议值。 */
    torque = J4310_FloatToUint(motor->direction * torque_nm, -J4310_TORQUE_MAX, J4310_TORQUE_MAX,
                               12U); /* J4310 前馈转矩编码后的 12 位协议值。 */

    applied_kp = kp;
    applied_kd = kd; /* Exact configured gains; POSITION passes both as zero. */
    kp_raw = J4310_FloatToUint(applied_kp, 0.0f, J4310_KP_MAX, 12U);
    kd_raw = J4310_FloatToUint(applied_kd, 0.0f, J4310_KD_MAX, 12U);
    memset(frame, 0, sizeof(*frame));
    frame->hfdcan = NULL;
    frame->id = J4310_GetControlId(motor);
    frame->length = 8U;
    frame->data[0] = (uint8_t)(position >> 8U);
    frame->data[1] = (uint8_t)position;
    frame->data[2] = (uint8_t)(speed >> 4U);
    frame->data[3] = (uint8_t)((speed << 4U) | (kp_raw >> 8U));
    frame->data[4] = (uint8_t)kp_raw;
    frame->data[5] = (uint8_t)(kd_raw >> 4U);
    frame->data[6] = (uint8_t)((kd_raw << 4U) | (torque >> 8U));
    frame->data[7] = (uint8_t)torque;
}

/* 速度模式的 T 型规划：以实测速度重起步，限加速度逼近目标。 */
static float J4310_PlanSpeed(j4310_t *motor, float target, uint32_t now)
{
    float dt = motor->speed_planner_valid ? (float)(uint32_t)(now - motor->speed_planner_tick) * 0.001f : 0.0f;

    if (!motor->speed_planner_valid || (dt > J4310_SPEED_PLANNER_RESEED_SEC))
    {
        Trapezoid_Reset(&motor->speed_planner, motor->feedback.speed_rad_s);
        dt = 0.0f;
    }
    else if (dt > J4310_SPEED_PLANNER_MAX_DT_SEC)
    {
        dt = J4310_SPEED_PLANNER_MAX_DT_SEC;
    }
    motor->speed_planner_tick = now;
    motor->speed_planner_valid = true;
    motor->speed_planner.a_acc = j4310_speed_max_acceleration_rad_s2;
    motor->speed_planner.a_dec = j4310_speed_max_acceleration_rad_s2;
    return Trapezoid_Update(&motor->speed_planner, target, dt);
}

/* 构造速度模式控制帧：CAN ID 为 0x200 + 节点号，载荷是目标速度的 32 位单精度浮点。 */
static void J4310_BuildVelocity(const j4310_t *motor, float speed_rad_s, can_frame_t *frame)
{
    float protocol_speed_rad_s = (float)motor->direction * speed_rad_s;
    uint32_t raw;

    memcpy(&raw, &protocol_speed_rad_s, sizeof(raw));
    memset(frame, 0, sizeof(*frame));
    frame->hfdcan = NULL;
    frame->id = motor->motor_id + J4310_SPEED_ID_OFFSET;
    frame->length = 8U;
    frame->data[0] = (uint8_t)(raw & 0xFFU);
    frame->data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    frame->data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    frame->data[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

/* 功能：在上电阶段周期重发使能帧，确认使能后持续生成锁定当前位置的 MIT 控制帧。 */
bool J4310_BuildControlFrame(j4310_t *motor /* 参与上电保持和正常控制的 J4310 对象。 */,
                             float additional_torque_nm /* 需要叠加的补偿转矩，单位：牛米。 */,
                             uint32_t now /* 用于采样五次位置轨迹的当前毫秒时刻。 */,
                             can_frame_t *frame /* 用于写出本周期 J4310 命令的 CAN 帧。 */)
{
    if (!motor || !frame)
        return false;
    if (motor->mode == J4310_MODE_STOP)
    {
        J4310_BuildDisable(motor, frame);
        return true;
    }

    if (motor->set_zero_position_request != 0U)
    {
        J4310_BuildSpecial(motor, J4310_SET_ZERO, frame);
        if (motor->feedback.received && fabsf(motor->feedback.position_rad) <= J4310_ZERO_POSITION_TOLERANCE_RAD)
        {
            motor->set_zero_position_request = 0U;
        }
        return true;
    }
    if (motor->fault_active)
    {
        if (motor->fault_disable_sent)
        {
            return false;
        }
        J4310_BuildDisable(motor, frame);
        motor->fault_disable_sent = true;
        return true;
    }

    if (motor->fault_recovery_pending)
    {
        if (motor->startup_enable_countdown > 0U)
        {
            motor->startup_enable_countdown--;
            return false;
        }
        J4310_BuildEnable(motor, frame);
        motor->startup_enable_countdown = J4310_STARTUP_ENABLE_RETRY_CYCLES - 1U;
        return true;
    }

    if (motor->startup_hold_pending)
    {
        if (motor->startup_enable_countdown > 0U)
        {
            motor->startup_enable_countdown--;
            return false;
        }

        J4310_BuildEnable(motor, frame);
        motor->startup_enable_countdown = J4310_STARTUP_ENABLE_RETRY_CYCLES - 1U;
        return true;
    }

    /* 速度模式：使用驱动器自带的速度模式帧（0x200 + 节点号），载荷是 float。 */
    if (motor->mode == J4310_MODE_SPEED)
    {
        J4310_BuildVelocity(motor, J4310_PlanSpeed(motor, motor->target_speed_rad_s, now), frame);
        return true;
    }

    if (motor->mode == J4310_MODE_POSITION)
    {
        float dt = (now - motor->cascade_tick) * 0.001f, speed_ref, tq;
        motor->cascade_tick = now;
        if (dt <= 0 || dt > 0.02f)
            dt = J4310_CASCADE_PERIOD_S;
        if (!motor->cascade_configured)
        {
            PID_Init(&motor->cascade_position_pid, J4310_POSITION_PID_KP, J4310_POSITION_PID_KI, J4310_POSITION_PID_KD,
                     J4310_POSITION_SPEED_LIMIT, J4310_POSITION_PID_I_LIMIT);
            PID_Init(&motor->cascade_speed_pid, J4310_SPEED_PID_KP, J4310_SPEED_PID_KI, J4310_SPEED_PID_KD,
                     J4310_TORQUE_MAX, J4310_SPEED_PID_I_LIMIT);
            motor->cascade_configured = true;
        }
        if (motor->trajectory_pending)
            J4310_BeginPositionTrajectory(motor, now);
        J4310_SamplePositionTrajectory(motor, now);
        speed_ref =
            PID_Calc(&motor->cascade_position_pid, motor->reference_position_rad, motor->feedback.position_rad, dt);
        tq = PID_Calc(&motor->cascade_speed_pid, speed_ref, motor->feedback.speed_rad_s, dt) + additional_torque_nm;
        J4310_BuildMit(motor, motor->feedback.position_rad, 0, 0, 0, tq, frame);
    }
    else if (motor->mode == J4310_MODE_DAMPING)
        J4310_BuildMit(motor, motor->feedback.position_rad, 0, 0, J4310_MIT_VELOCITY_KD, additional_torque_nm, frame);
    else if (motor->mode == J4310_MODE_MIT)
        J4310_BuildMit(motor, motor->target_position_rad, 0, J4310_MIT_POSITION_KP, J4310_MIT_VELOCITY_KD,
                       motor->target_torque_nm, frame);
    else
        return false;
    return true;
}

/* Three-argument mode/target interface; BR managed objects source gains from arm_config.h. */
void J4310_SetTargetBasic(j4310_t *motor, j4310_mode_t mode, float target)
{
    if (!motor || !isfinite(target) || mode < J4310_MODE_STOP || mode >= J4310_MODE_MIT)
        return;
    if (motor->target_sink)
    {
        motor->target_sink(motor, mode, target, 0);
        return;
    }
    if (motor->mode != mode)
    {
        PID_Reset(&motor->cascade_position_pid);
        PID_Reset(&motor->cascade_speed_pid);
        motor->cascade_tick = HAL_GetTick();
    }
    if (mode == J4310_MODE_POSITION)
    {
        float p = target * J4310_DEG_TO_RAD;
        if (fabsf(p) > J4310_POSITION_MAX)
            return;
        if (motor->mode != mode || fabsf(p - motor->target_position_rad) > J4310_TRAJECTORY_EPSILON_RAD)
            motor->trajectory_pending = true;
        motor->target_position_rad = p;
        motor->startup_hold_active = true;
    }
    else
    {
        motor->target_speed_rad_s = mode == J4310_MODE_SPEED ? target : 0;
        motor->trajectory_pending = false;
        motor->trajectory_active = false;
        motor->startup_hold_active = false;
    }
    motor->target_torque_nm = 0;
    motor->mode = mode;
}
void J4310_SetTargetMIT(j4310_t *motor, j4310_mode_t mode, float tau_nm, float output_degrees)
{
    if (!motor || mode != J4310_MODE_MIT || !isfinite(tau_nm) || !isfinite(output_degrees))
        return;
    if (motor->target_sink)
    {
        motor->target_sink(motor, mode, tau_nm, output_degrees);
        return;
    }
    if (fabsf(tau_nm) > J4310_TORQUE_MAX || fabsf(output_degrees * J4310_DEG_TO_RAD) > J4310_POSITION_MAX)
        return;
    motor->target_position_rad = motor->direction * output_degrees * J4310_DEG_TO_RAD - motor->position_offset_rad;
    motor->target_torque_nm = motor->direction * tau_nm; /* BuildMit converts back to physical motor sign. */
    motor->mode = J4310_MODE_MIT;
    motor->trajectory_pending = false;
    motor->trajectory_active = false;
    PID_Reset(&motor->cascade_position_pid);
    PID_Reset(&motor->cascade_speed_pid);
}
