/*
 * @file rs00.c
 * @brief 实现 RS00 电机的扩展 CAN 控制、反馈解码和目标管理。
 */

/* RS00 电机驱动实现：完成 MIT 风格命令编码和反馈解析。 */
#include "rs00.h"
#include "bsp_can.h"
#include "flash.h"
#include "quintic_planner.h"
#include "rs05.h"
#include "trapezoid_planner.h"
#include <math.h>
#include <string.h>

extern volatile uint8_t g_motor_power_recovery_enable;
extern volatile uint8_t g_rs_recovery_motion_release;

volatile float rs00_trajectory_max_speed_rad_s = 10.0f;
volatile float rs00_trajectory_max_acceleration_rad_s2 = 30.0f;
/* 0 = 限幅优先（按最大速度/加速度反解时长）；>0 = 时间优先（按该时长正解峰值）。 */
volatile float rs00_trajectory_time_s = 0.0f;
/* 速度模式的加速度上限：默认与五次轨迹同值，按负载惯量在 变量观察窗口 里调。 */
volatile float rs00_speed_max_acceleration_rad_s2 = 30.0f;
/* 速度模式电流限制与电机内部加速度：默认给保守值，按实际负载标定。 */
volatile float rs00_speed_limit_current_a = 8.0f;
volatile float rs00_speed_accel_rad_s2 = 30.0f;

/* 将 RS00 浮点控制量限制在指定范围内。 */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 将输入物理量限制在给定范围内，并编码为协议 16 位无符号字段。 */
static uint16_t enc(float v, float lo, float hi)
{
    v = clampf(v, lo, hi);
    return (uint16_t)((v - lo) * 65535.0f / (hi - lo));
}

/* 将协议 16 位无符号字段按给定范围解码为物理量。 */
static float dec(uint16_t v, float lo, float hi)
{
    return lo + (float)v * (hi - lo) / 65535.0f;
}

/* 读取大端序 16 位字段。 */
static uint16_t rdbe(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

/* 组装 RS00 扩展 CAN 标识符。 */
static uint32_t pack(uint8_t cmd, uint16_t extra, uint8_t id)
{
    return ((uint32_t)cmd << 24) | ((uint32_t)extra << 8) | id;
}

/* 用通用五次规划器生成本段轨迹：静止起动 k0=0，运行中换目标则延续实测速度。 */
static void RS00_PlanTrajectory(rs00_t *m, float start, float target)
{
    quintic_plan_t plan;
    float duration_ms;

    /* 规划入口：给角度和时长，算出这段运动需要的峰值速度/加速度。 */
    Quintic_SetDefaultLimits(rs00_trajectory_max_speed_rad_s, rs00_trajectory_max_acceleration_rad_s2,
                             (float)RS00_TRAJECTORY_PERIOD_MS * 0.001f);
    Quintic_Plan(&plan, start, target, m->velocity, rs00_trajectory_time_s);
    m->trajectory_start_position = start;
    m->trajectory_delta_position = target - start;
    m->trajectory_k0 = plan.k0;
    duration_ms = plan.duration * 1000.0f;
    if (duration_ms > 0.0f)
    {
        duration_ms = ceilf(duration_ms / (float)RS00_TRAJECTORY_PERIOD_MS) * (float)RS00_TRAJECTORY_PERIOD_MS;
    }
    m->trajectory_duration_ms = (uint32_t)duration_ms;
}

/* 以最新实测位置和实测速度为起点建立五次位置轨迹。 */
static void RS00_BeginPositionTrajectory(rs00_t *m, uint32_t now)
{
    RS00_PlanTrajectory(m, m->position, m->target);
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
        /* 首帧前馈速度就是本段起始速度：静止起动为 0，中途换目标为实测速度。 */
        m->reference_velocity =
            m->trajectory_delta_position * m->trajectory_k0 / ((float)m->trajectory_duration_ms * 0.001f);
    }
}

/* 按当前时刻采样五次轨迹的位置和速度（k0 为 0 时就是 10t^3-15t^4+6t^5）。 */
static void RS00_UpdatePositionTrajectory(rs00_t *m, uint32_t now)
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

/* 发送一帧 RS00 命令。 */
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

/* 通信类型 18：写 32 位浮点参数。 */
static bool RS00_WriteParamFloat(rs00_t *m, uint16_t index, float value)
{
    uint8_t data[8] = {0};

    memcpy(&data[0], &index, 2U);
    memcpy(&data[4], &value, 4U);
    return tx(m->hcan, RS00_CMD_WRITE_PARAM, RS00_HOST_ID, m->id, data) == HAL_OK;
}

/* 通信类型 18：写 1 字节参数（run_mode 用）。 */
static bool RS00_WriteParamByte(rs00_t *m, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {0};

    memcpy(&data[0], &index, 2U);
    data[4] = value;
    return tx(m->hcan, RS00_CMD_WRITE_PARAM, RS00_HOST_ID, m->id, data) == HAL_OK;
}

/* 切回运控（MIT）模式，只在模式真正变化时下发一次。 */
static bool RS00_EnsureMitMode(rs00_t *m)
{
    if (m->run_mode == RS00_RUN_MODE_MIT)
    {
        return true;
    }
    if (!RS00_WriteParamByte(m, RS00_IDX_RUN_MODE, RS00_RUN_MODE_MIT))
    {
        return false;
    }
    m->run_mode = RS00_RUN_MODE_MIT;
    return true;
}

/* 切到速度模式，并下发速度模式需要的电流限制和电机内部加速度。 */
static bool RS00_EnsureSpeedMode(rs00_t *m)
{
    if (m->run_mode == RS00_RUN_MODE_SPEED)
    {
        return true;
    }
    if (!RS00_WriteParamByte(m, RS00_IDX_RUN_MODE, RS00_RUN_MODE_SPEED))
    {
        return false;
    }
    (void)RS00_WriteParamFloat(m, RS00_IDX_LIMIT_CUR, rs00_speed_limit_current_a);
    (void)RS00_WriteParamFloat(m, RS00_IDX_ACC_RAD, rs00_speed_accel_rad_s2);
    m->run_mode = RS00_RUN_MODE_SPEED;
    return true;
}

/* 发送零增益运动帧以触发一帧反馈，不产生位置、速度或转矩控制输出。 */
static bool RS00_PollFeedback(rs00_t *m)
{
    uint8_t data[8];
    uint16_t zero_torque;

    if (m == NULL || !m->enabled)
    {
        return false;
    }

    data[0] = (uint8_t)(enc(0.0f, RS00_POSITION_MIN, RS00_POSITION_MAX) >> 8);
    data[1] = (uint8_t)enc(0.0f, RS00_POSITION_MIN, RS00_POSITION_MAX);
    data[2] = (uint8_t)(enc(0.0f, -RS00_VELOCITY_MAX, RS00_VELOCITY_MAX) >> 8);
    data[3] = (uint8_t)enc(0.0f, -RS00_VELOCITY_MAX, RS00_VELOCITY_MAX);
    data[4] = (uint8_t)(enc(0.0f, 0.0f, 500.0f) >> 8); /* Kp = 0。 */
    data[5] = (uint8_t)enc(0.0f, 0.0f, 500.0f);
    data[6] = (uint8_t)(enc(0.0f, 0.0f, 5.0f) >> 8); /* Kd = 0。 */
    data[7] = (uint8_t)enc(0.0f, 0.0f, 5.0f);
    zero_torque = enc(0.0f, -RS00_TORQUE_MAX, RS00_TORQUE_MAX);

    return tx(m->hcan, RS00_CMD_MOTION, zero_torque, m->id, data) == HAL_OK;
}

/* 向 RS00 发送设置当前位置为零点的命令。 */
static bool RS00_SendSetZero(rs00_t *m)
{
    uint8_t data[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    return (m != NULL) && (tx(m->hcan, RS00_SET_ZERO, RS00_HOST_ID, m->id, data) == HAL_OK);
}

/* 构造 RS00 特殊命令帧。 */
bool RS00_BuildSpecial(rs00_t *m, uint8_t command)
{
    if (m != NULL && command == RS00_FALSE)
    {
        m->startup_hold_active = false;
    }
    return (m != NULL) && (tx(m->hcan, command, RS00_HOST_ID, m->id, NULL) == HAL_OK);
}

/* 初始化 RS00 节点编号、总线状态和默认目标。 */
HAL_StatusTypeDef RS00_Init(rs00_t *m, FDCAN_HandleTypeDef *hcan, uint8_t id, int direction)
{
    if (m == NULL || hcan == NULL || id == 0U)
    {
        return HAL_ERROR;
    }
    memset(m, 0, sizeof(*m));
    m->hcan = hcan;
    m->id = id;
    m->direction = (direction < 0) ? -1 : 1;
    m->mode = RS_MODE_STOP;
    m->continuous_valid = Flash_LoadMotorAngle(0U, &m->continuous_position);
    m->enabled = RS00_BuildSpecial(m, RS00_ENABLE);
    return HAL_OK;
}

/* 停止 RS00 并清零控制目标。 */
bool RS00_Stop(rs00_t *m)
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
    sent = RS00_BuildSpecial(m, RS00_FALSE);
    m->enabled = false;
    return sent;
}

/* 设置 RS00 运行模式及目标值。 */
void RS00_SetTargetBasic(rs00_t *m, rs_mode_t mode, float target)
{
    if (mode == RS_MODE_MIT)
        return; /* MIT requires tau and angle. */

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
        position_target = clampf(target * RS00_DEG_TO_RAD, RS00_POSITION_MIN, RS00_POSITION_MAX);
        target_changed = (m->mode != RS_MODE_POSITION) || m->recovery_trajectory ||
                         (fabsf(position_target - m->target) > RS00_TRAJECTORY_EPSILON_RAD);
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
        m->target = clampf(target, -RS00_VELOCITY_MAX, RS00_VELOCITY_MAX);
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

/* 速度模式的 T 型规划：限加速度地把目标速度逼近，避免速度模式出现命令阶跃。
   首次调用或距离上次调用过久时，以实测速度为起点重新起步，不产生跳变。 */
static float RS00_PlanSpeed(rs00_t *m, float target)
{
    uint32_t now = HAL_GetTick();
    float dt = m->speed_planner_valid ? (float)(uint32_t)(now - m->speed_planner_tick) * 0.001f : 0.0f;

    if (!m->speed_planner_valid || (dt > RS00_SPEED_PLANNER_RESEED_SEC))
    {
        Trapezoid_Reset(&m->speed_planner, m->velocity);
        dt = 0.0f;
    }
    else if (dt > RS00_SPEED_PLANNER_MAX_DT_SEC)
    {
        dt = RS00_SPEED_PLANNER_MAX_DT_SEC;
    }
    m->speed_planner_tick = now;
    m->speed_planner_valid = true;
    m->speed_planner.a_acc = rs00_speed_max_acceleration_rad_s2;
    m->speed_planner.a_dec = rs00_speed_max_acceleration_rad_s2;
    return Trapezoid_Update(&m->speed_planner, target, dt);
}

/* 编码并发送当前 RS00 运动目标。 */
bool RS00_Send(rs00_t *m)
{
    uint8_t d[8];
    float pos, vel = 0, tq = 0, kp = 0, kd = 0;
    uint32_t now;
    float dt;
    if (!m || !m->feedback_valid)
        return false;
    if (m->mode == RS_MODE_STOP)
        return RS00_Stop(m);
    if (m->mode == RS_MODE_SPEED)
    {
        if (!RS00_EnsureSpeedMode(m))
            return false;
        return RS00_WriteParamFloat(m, RS00_IDX_SPD_REF, m->direction * RS00_PlanSpeed(m, m->target));
    }
    if (!RS00_EnsureMitMode(m))
        return false;
    now = HAL_GetTick();
    dt = (now - m->cascade_tick) * 0.001f;
    m->cascade_tick = now;
    if (dt <= 0 || dt > 0.02f)
        dt = RS00_CASCADE_PERIOD_S;
    if (!m->cascade_configured)
    {
        PID_Init(&m->cascade_position_pid, RS00_POSITION_PID_KP, RS00_POSITION_PID_KI, RS00_POSITION_PID_KD,
                 RS00_POSITION_SPEED_LIMIT, RS00_POSITION_PID_I_LIMIT);
        PID_Init(&m->cascade_speed_pid, RS00_SPEED_PID_KP, RS00_SPEED_PID_KI, RS00_SPEED_PID_KD, RS00_TORQUE_MAX,
                 RS00_SPEED_PID_I_LIMIT);
        m->cascade_configured = true;
    }
    pos = m->position;
    if (m->mode == RS_MODE_POSITION)
    {
        float speed_ref;
        if (m->trajectory_pending)
            RS00_BeginPositionTrajectory(m, now);
        RS00_UpdatePositionTrajectory(m, now);
        speed_ref = PID_Calc(&m->cascade_position_pid, m->reference_position, m->position, dt);
        tq = m->direction * PID_Calc(&m->cascade_speed_pid, speed_ref, m->velocity, dt);
        /* KP=KD=0: external position->speed cascade supplies torque. */
    }
    else if (m->mode == RS_MODE_DAMPING)
    {
        kd = RS00_MIT_VELOCITY_KD;
    }
    else if (m->mode == RS_MODE_MIT)
    {
        pos = m->target;
        kp = RS00_MIT_POSITION_KP;
        kd = RS00_MIT_VELOCITY_KD;
        tq = m->tau_ff_nm;
    }
    else
        return false;
    pos *= m->direction;
    {
        uint16_t p = enc(pos, RS00_POSITION_MIN, RS00_POSITION_MAX);
        uint16_t v = enc(vel, -RS00_VELOCITY_MAX, RS00_VELOCITY_MAX);
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
    return tx(m->hcan, RS00_CMD_MOTION, enc(tq, -RS00_TORQUE_MAX, RS00_TORQUE_MAX), m->id, d) == HAL_OK;
}

/* 解析 RS00 反馈帧并刷新在线状态。 */
bool RS00_OnFrame(rs00_t *m, const can_frame_t *f)
{
    uint8_t cmd;
    bool recovering; // 标记当前是否为反馈恢复后的第一帧

    // 检查对象、CAN 外设、帧类型和数据长度
    if (!m || !f || f->hfdcan != m->hcan || f->id_type != FDCAN_EXTENDED_ID || f->length != 8U)
    {
        return false;
    }

    // 从扩展 CAN ID 中提取命令类型
    cmd = (uint8_t)((f->id >> 24) & 0x1FU);

    // 检查命令类型、主机 ID 和电机 ID 是否匹配
    if (cmd != RS00_CMD_FEEDBACK || (uint8_t)f->id != RS00_HOST_ID || (uint8_t)(f->id >> 8) != m->id)
    {
        return false;
    }

    // 从 CAN ID 中解析电机故障码
    m->fault = (uint8_t)((f->id >> 16) & 0x3FU);

    // 解析单圈位置，并根据电机方向修正符号
    m->single_position = (float)m->direction * dec(rdbe(&f->data[0]), RS00_POSITION_MIN, RS00_POSITION_MAX);

    // 解析速度，并根据电机方向修正符号
    m->velocity = (float)m->direction * dec(rdbe(&f->data[2]), -RS00_VELOCITY_MAX, RS00_VELOCITY_MAX);

    // 解析力矩，并根据电机方向修正符号
    m->torque = (float)m->direction * dec(rdbe(&f->data[4]), -RS00_TORQUE_MAX, RS00_TORQUE_MAX);

    // 判断当前是否为通信恢复后的第一帧反馈
    recovering = !m->feedback_valid;

    // 保存处理当前报文前的离线状态

    // 首次恢复反馈时，初始化位置和反馈状态
    if (recovering)
    {
        // 连续位置尚未初始化时，使用当前单圈位置作为初始值
        if (!m->continuous_valid)
        {
            m->continuous_position = m->single_position;
        }

        // 标记连续位置、反馈和电机状态均已恢复
        m->continuous_valid = true;
        m->feedback_valid = true;
        m->feedback_recovery_pending = false;
        m->enabled = true;
        m->ever_feedback_received = true;
    }

    // 根据单圈位置计算不会跨周期跳变的连续位置
    if (m->continuous_valid)
    {
        m->continuous_position +=
            remainderf(m->single_position - remainderf(m->continuous_position, RS00_ANGLE_PERIOD), RS00_ANGLE_PERIOD);
    }

    // 保存当前连续位置
    m->position = m->continuous_position;

    // 通信恢复后，重新初始化位置控制相关状态
    if (recovering)
    {
        // 以当前实际位置作为保持位置和参考位置
        m->hold_position = m->position;
        m->reference_position = m->position;

        // 恢复时将参考速度清零
        m->reference_velocity = 0.0f;

        // 根据当前模式和启动保持状态决定是否等待轨迹恢复
        m->trajectory_pending = (m->mode == RS_MODE_POSITION && m->startup_hold_active);

        // 停止当前正在执行的轨迹
        m->trajectory_active = false;

        // 不需要恢复轨迹时，将目标位置设置为当前位置
        if (!m->trajectory_pending)
        {
            m->target = m->position;
        }

        // 首次上电也执行回零轨迹。
        if (!m->startup_hold_initialized)
        {
            m->recovery_trajectory = (g_motor_power_recovery_enable == 1U);
            m->mode = m->recovery_trajectory ? RS_MODE_POSITION : RS_MODE_STOP;
            m->target = m->recovery_trajectory ? 0.0f : m->position;
            m->startup_hold_active = m->recovery_trajectory;
            m->trajectory_pending = m->recovery_trajectory;
            m->trajectory_active = false;
            m->startup_hold_initialized = true;
        }

        // 电机从离线恢复且允许自动恢复时，重新设置恢复目标
        if (g_motor_power_recovery_enable == 1U)
        {
            m->recovery_trajectory = true;
            m->mode = RS_MODE_POSITION;
            m->target = 0.0f;
            m->trajectory_pending = true;
            m->trajectory_active = false;
            m->startup_hold_active = true;
        }
    }

    // 清除离线标志，并记录最后一次反馈时间
    m->offline_latched = false;
    m->last_feedback_tick = HAL_GetTick();

    // 报文校验并处理成功
    return true;
}

/* 读取 RS00 连续位置并转换为角度输出。 */
void RS00_GetPositionDegrees(const rs00_t *m, volatile float *position_degrees)
{
    if (position_degrees != NULL)
    {
        *position_degrees = (m == NULL) ? 0.0f : m->position * 57.2957795131f;
    }
}

/* 周期维护 RS00 零点保存、反馈超时和运动控制帧。 */
void RS00_Service(rs00_t *m)
{
    uint32_t now;

    // 电机对象为空时直接返回
    if (!m)
    {
        return;
    }

    // 获取当前系统时间
    now = HAL_GetTick();

    if (m->recovery_trajectory && g_motor_power_recovery_enable != 1U)
    {
        m->recovery_trajectory = false;
        m->mode = RS_MODE_STOP;
        m->target = m->position;
        m->trajectory_pending = false;
        m->trajectory_active = false;
        m->startup_hold_active = false;
    }

    // 处理设置电机零点请求
    if (m->set_zero_position_request != 0U)
    {
        // 发送设置零点命令
        (void)RS00_SendSetZero(m);

        // 收到有效反馈且当前位置接近零点时，清除设置请求
        if (m->feedback_valid && fabsf(m->single_position) <= RS00_ZERO_POSITION_TOLERANCE_RAD)
        {
            m->set_zero_position_request = 0U;
        }

        // 设置零点期间不执行后续控制流程
        return;
    }

    /* 不使用反馈超时看门狗；持续按当前目标发送控制帧。 */

    if (m->recovery_trajectory && g_rs_recovery_motion_release == 0U)
    {
        return;
    }

    if (m->mode == RS_MODE_POSITION || m->mode == RS_MODE_SPEED || m->mode == RS_MODE_DAMPING || m->mode == RS_MODE_MIT)
    {
        if ((uint32_t)(now - m->last_control_tick) >= 5U)
        {
            m->last_control_tick = now;
            (void)RS00_Send(m);
        }
    }
}

void RS00_SetTargetMIT(rs00_t *motor, rs_mode_t mode, float tau_nm, float output_degrees)
{
    if (!motor || mode != RS_MODE_MIT || !isfinite(tau_nm) || !isfinite(output_degrees))
        return;
    if (motor->target_sink)
    {
        motor->target_sink(motor, (int)mode, tau_nm, output_degrees);
        return;
    }
    if (fabsf(output_degrees * RS00_DEG_TO_RAD) > RS00_POSITION_MAX || fabsf(tau_nm) > RS00_TORQUE_MAX)
        return;
    motor->mode = RS_MODE_MIT;
    motor->target = motor->direction * output_degrees * RS00_DEG_TO_RAD;
    motor->tau_ff_nm = tau_nm;
    motor->trajectory_pending = false;
    motor->trajectory_active = false;
    motor->startup_hold_active = true;
    motor->recovery_trajectory = false;
    PID_Reset(&motor->cascade_position_pid);
    PID_Reset(&motor->cascade_speed_pid);
}
