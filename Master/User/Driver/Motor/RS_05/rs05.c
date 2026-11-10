/*
 * @file rs05.c
 * @brief 实现 RS05 电机的扩展 CAN 控制、反馈解码和目标管理。
 */

/* RS05 电机驱动实现：完成 MIT 风格命令编码和反馈解析。 */
#include "rs05.h"
#include "bsp_can.h"
#include "flash.h"
#include "quintic_planner.h"
#include "trapezoid_planner.h"
#include <math.h>
#include <string.h>

extern volatile uint8_t g_motor_power_recovery_enable;
extern volatile uint8_t g_rs_recovery_motion_release;

volatile float rs05_trajectory_max_speed_rad_s = 10.0f;
volatile float rs05_trajectory_max_acceleration_rad_s2 = 30.0f;
/* 0 = 限幅优先（按最大速度/加速度反解时长）；>0 = 时间优先（按该时长正解峰值）。 */
volatile float rs05_trajectory_time_s = 0.0f;
/* 速度模式的加速度上限：默认与五次轨迹同值，按负载惯量在 变量观察窗口 里调。 */
volatile float rs05_speed_max_acceleration_rad_s2 = 30.0f;
/* 速度模式电流限制与电机内部加速度：默认给保守值，按实际负载标定。 */
volatile float rs05_speed_limit_current_a = 8.0f;
volatile float rs05_speed_accel_rad_s2 = 30.0f;

/* 自动回零速度规划：巡航 2.5 rad/s（原 5.0 的一半），加速度 0.8333 rad/s^2（原 3.3333 的 1/4）。
   两者同时缩小，任意行程的速度-时间曲线幅度都是原值的一半，即真实回零转速减半、用时翻倍；
   只减巡航速度时，短行程仍由加速度决定峰值，真实转速不会变。 */
volatile float rs_recovery_max_speed_rad_s = 2.5f;
volatile float rs_recovery_max_acceleration_rad_s2 = 0.8333333333f;

/* 将浮点控制量限制在指定范围内。 */
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

/* 组装 RS05 扩展 CAN 标识符。 */
static uint32_t pack(uint8_t cmd, uint16_t extra, uint8_t id)
{
    return ((uint32_t)cmd << 24) | ((uint32_t)extra << 8) | id;
}

/* 用通用五次规划器生成本段轨迹：静止起动 k0=0，运行中换目标则延续实测速度。
   时长向上取整到控制周期整数倍，保证采样点落在 2ms 网格上。 */
static void RS05_PlanTrajectory(rs05_t *m, float start, float target)
{
    quintic_plan_t plan;
    float duration_ms;

    /* 规划入口：给角度和时长，算出这段运动需要的峰值速度/加速度。 */
    Quintic_SetDefaultLimits(rs05_trajectory_max_speed_rad_s, rs05_trajectory_max_acceleration_rad_s2,
                             (float)RS05_TRAJECTORY_PERIOD_MS * 0.001f);
    Quintic_Plan(&plan, start, target, m->velocity, rs05_trajectory_time_s);
    m->trajectory_start_position = start;
    m->trajectory_delta_position = target - start;
    m->trajectory_k0 = plan.k0;
    duration_ms = plan.duration * 1000.0f;
    if (duration_ms > 0.0f)
    {
        duration_ms = ceilf(duration_ms / (float)RS05_TRAJECTORY_PERIOD_MS) * (float)RS05_TRAJECTORY_PERIOD_MS;
    }
    m->trajectory_duration_ms = (uint32_t)duration_ms;
}

/* 共用巡航速度；制动取决于各电机的剩余行程。 */
float RS05_RecoveryVelocity(float error_rad, float previous_rad_s, float actual_rad_s, float dt_s)
{
    float speed = rs_recovery_max_speed_rad_s;
    float acceleration = rs_recovery_max_acceleration_rad_s2;
    float desired;
    float step;

    if (!isfinite(error_rad) || !isfinite(previous_rad_s) || !isfinite(actual_rad_s) || !isfinite(speed) ||
        !isfinite(acceleration) || speed <= 0.0f || acceleration <= 0.0f || !isfinite(dt_s))
    {
        return 0.0f;
    }
    /* 速度参考值设为零，使 Kd 对任一方向的实测超速产生制动作用。
       此处为反馈制动，并非驱动器端的硬性速度限制。 */
    if (fabsf(actual_rad_s) >= speed)
    {
        return 0.0f;
    }
    dt_s = clampf(dt_s, 0.0f, 0.02f);
    desired = fminf(speed, sqrtf(2.0f * acceleration * fabsf(error_rad)));
    desired = fminf(desired, 4.0f * fabsf(error_rad));
    desired = copysignf(desired, error_rad);
    step = acceleration * dt_s;
    return clampf(previous_rad_s + clampf(desired - previous_rad_s, -step, step), -speed, speed);
}

/* 以最新实测位置和实测速度为起点建立五次位置轨迹。 */
static void RS05_BeginPositionTrajectory(rs05_t *m, uint32_t now)
{
    RS05_PlanTrajectory(m, m->position, m->target);
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
static void RS05_UpdatePositionTrajectory(rs05_t *m, uint32_t now)
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

/* 发送一帧 RS05 命令。 */
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

/* 发送零增益运动帧以触发一帧反馈，不产生位置、速度或转矩控制输出。 */
static bool RS05_PollFeedback(rs05_t *m)
{
    uint8_t data[8];
    uint16_t zero_torque;

    if (m == NULL || !m->enabled)
    {
        return false;
    }

    data[0] = (uint8_t)(enc(0.0f, RS05_POSITION_MIN, RS05_POSITION_MAX) >> 8);
    data[1] = (uint8_t)enc(0.0f, RS05_POSITION_MIN, RS05_POSITION_MAX);
    data[2] = (uint8_t)(enc(0.0f, -RS05_VELOCITY_MAX, RS05_VELOCITY_MAX) >> 8);
    data[3] = (uint8_t)enc(0.0f, -RS05_VELOCITY_MAX, RS05_VELOCITY_MAX);
    data[4] = (uint8_t)(enc(0.0f, RS05_KP_MIN, RS05_KP_MAX) >> 8); /* Kp = 0。 */
    data[5] = (uint8_t)enc(0.0f, RS05_KP_MIN, RS05_KP_MAX);
    data[6] = (uint8_t)(enc(0.0f, RS05_KD_MIN, RS05_KD_MAX) >> 8); /* Kd = 0。 */
    data[7] = (uint8_t)enc(0.0f, RS05_KD_MIN, RS05_KD_MAX);
    zero_torque = enc(0.0f, -RS05_TORQUE_MAX, RS05_TORQUE_MAX);
    return tx(m->hcan, RS05_CMD_MOTION, zero_torque, m->id, data) == HAL_OK;
}

/* 向 RS05 发送设置当前位置为零点的命令。 */
static bool RS05_SendSetZero(rs05_t *m)
{
    uint8_t data[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    return (m != NULL) && (tx(m->hcan, RS05_SET_ZERO, RS05_HOST_ID, m->id, data) == HAL_OK);
}

/* 构造 RS05 特殊命令帧。 */
bool RS05_BuildSpecial(rs05_t *m, uint8_t command)
{
    if (m != NULL && command == RS05_FALSE)
    {
        m->startup_hold_active = false;
    }
    return (m != NULL) && (tx(m->hcan, command, RS05_HOST_ID, m->id, NULL) == HAL_OK);
}

/* 初始化 RS05 节点编号、总线状态和默认目标。 */
HAL_StatusTypeDef RS05_Init(rs05_t *m, FDCAN_HandleTypeDef *hcan, uint8_t id, int direction)
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
    m->continuous_valid = Flash_LoadMotorAngle(1U, &m->continuous_position);
    m->enabled = RS05_BuildSpecial(m, RS05_ENABLE);
    return HAL_OK;
}

/* 停止 RS05 并清零控制目标。 */
bool RS05_Stop(rs05_t *m)
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
    sent = RS05_BuildSpecial(m, RS05_FALSE);
    m->enabled = false;
    return sent;
}

/* 设置 RS05 运行模式及目标值。 */
void RS05_SetTargetBasic(rs05_t *m, rs_mode_t mode, float target)
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
        position_target = clampf(target * RS05_DEG_TO_RAD, RS05_POSITION_MIN, RS05_POSITION_MAX);
        target_changed = (m->mode != RS_MODE_POSITION) || m->recovery_trajectory ||
                         (fabsf(position_target - m->target) > RS05_TRAJECTORY_EPSILON_RAD);
        m->mode = mode;
        m->target = position_target;
        /* 首帧不保持；只有用户明确下发位置目标后才持续输出位置控制。 */
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
        m->target = clampf(target, -RS05_VELOCITY_MAX, RS05_VELOCITY_MAX);
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
    /* SetTarget 只更新目标；实际运动帧由 RS05_Service 周期发送。 */
}

/* 速度模式的 T 型规划：限加速度地把目标速度逼近，避免速度模式出现命令阶跃。
   首次调用或距离上次调用过久时，以实测速度为起点重新起步，不产生跳变。 */
static float RS05_PlanSpeed(rs05_t *m, float target)
{
    uint32_t now = HAL_GetTick();
    float dt = m->speed_planner_valid ? (float)(uint32_t)(now - m->speed_planner_tick) * 0.001f : 0.0f;

    if (!m->speed_planner_valid || (dt > RS05_SPEED_PLANNER_RESEED_SEC))
    {
        Trapezoid_Reset(&m->speed_planner, m->velocity);
        dt = 0.0f;
    }
    else if (dt > RS05_SPEED_PLANNER_MAX_DT_SEC)
    {
        dt = RS05_SPEED_PLANNER_MAX_DT_SEC;
    }
    m->speed_planner_tick = now;
    m->speed_planner_valid = true;
    m->speed_planner.a_acc = rs05_speed_max_acceleration_rad_s2;
    m->speed_planner.a_dec = rs05_speed_max_acceleration_rad_s2;
    return Trapezoid_Update(&m->speed_planner, target, dt);
}

/* 通信类型 18：写 32 位浮点参数。 */
static bool RS05_WriteParamFloat(rs05_t *m, uint16_t index, float value)
{
    uint8_t data[8] = {0};

    memcpy(&data[0], &index, 2U);
    memcpy(&data[4], &value, 4U);
    return tx(m->hcan, RS05_CMD_WRITE_PARAM, RS05_HOST_ID, m->id, data) == HAL_OK;
}

/* 通信类型 18：写 1 字节参数（run_mode 用）。 */
static bool RS05_WriteParamByte(rs05_t *m, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {0};

    memcpy(&data[0], &index, 2U);
    data[4] = value;
    return tx(m->hcan, RS05_CMD_WRITE_PARAM, RS05_HOST_ID, m->id, data) == HAL_OK;
}

/* 切回运控（MIT）模式，只在模式真正变化时下发一次。 */
static bool RS05_EnsureMitMode(rs05_t *m)
{
    if (m->run_mode == RS05_RUN_MODE_MIT)
    {
        return true;
    }
    if (!RS05_WriteParamByte(m, RS05_IDX_RUN_MODE, RS05_RUN_MODE_MIT))
    {
        return false;
    }
    m->run_mode = RS05_RUN_MODE_MIT;
    return true;
}

/* 切到速度模式，并下发速度模式需要的电流限制和电机内部加速度。 */
static bool RS05_EnsureSpeedMode(rs05_t *m)
{
    if (m->run_mode == RS05_RUN_MODE_SPEED)
    {
        return true;
    }
    if (!RS05_WriteParamByte(m, RS05_IDX_RUN_MODE, RS05_RUN_MODE_SPEED))
    {
        return false;
    }
    (void)RS05_WriteParamFloat(m, RS05_IDX_LIMIT_CUR, rs05_speed_limit_current_a);
    (void)RS05_WriteParamFloat(m, RS05_IDX_ACC_RAD, rs05_speed_accel_rad_s2);
    m->run_mode = RS05_RUN_MODE_SPEED;
    return true;
}

/* 编码并发送当前 RS05 运动目标。 */
bool RS05_Send(rs05_t *m)
{
    uint8_t d[8];
    float pos, vel = 0, tq = 0, kp = 0, kd = 0;
    uint32_t now;
    float dt;
    if (!m || !m->feedback_valid)
        return false;
    if (m->mode == RS_MODE_STOP)
        return RS05_Stop(m);
    if (m->mode == RS_MODE_SPEED)
    {
        if (!RS05_EnsureSpeedMode(m))
            return false;
        return RS05_WriteParamFloat(m, RS05_IDX_SPD_REF, m->direction * RS05_PlanSpeed(m, m->target));
    }
    if (!RS05_EnsureMitMode(m))
        return false;
    now = HAL_GetTick();
    dt = (now - m->cascade_tick) * 0.001f;
    m->cascade_tick = now;
    if (dt <= 0 || dt > 0.02f)
        dt = RS05_CASCADE_PERIOD_S;
    if (!m->cascade_configured)
    {
        PID_Init(&m->cascade_position_pid, RS05_POSITION_PID_KP, RS05_POSITION_PID_KI, RS05_POSITION_PID_KD,
                 RS05_POSITION_SPEED_LIMIT, RS05_POSITION_PID_I_LIMIT);
        PID_Init(&m->cascade_speed_pid, RS05_SPEED_PID_KP, RS05_SPEED_PID_KI, RS05_SPEED_PID_KD, RS05_TORQUE_MAX,
                 RS05_SPEED_PID_I_LIMIT);
        m->cascade_configured = true;
    }
    pos = m->position;
    if (m->mode == RS_MODE_POSITION)
    {
        float speed_ref;
        if (m->trajectory_pending)
            RS05_BeginPositionTrajectory(m, now);
        RS05_UpdatePositionTrajectory(m, now);
        speed_ref = PID_Calc(&m->cascade_position_pid, m->reference_position, m->position, dt);
        tq = m->direction * PID_Calc(&m->cascade_speed_pid, speed_ref, m->velocity, dt);
        /* KP=KD=0: external position->speed cascade supplies torque. */
    }
    else if (m->mode == RS_MODE_DAMPING)
    {
        kd = RS05_MIT_VELOCITY_KD;
    }
    else if (m->mode == RS_MODE_MIT)
    {
        pos = m->target;
        kp = RS05_MIT_POSITION_KP;
        kd = RS05_MIT_VELOCITY_KD;
        tq = m->tau_ff_nm;
    }
    else
        return false;
    pos *= m->direction;
    {
        uint16_t p = enc(pos, RS05_POSITION_MIN, RS05_POSITION_MAX);
        uint16_t v = enc(vel, -RS05_VELOCITY_MAX, RS05_VELOCITY_MAX);
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
    return tx(m->hcan, RS05_CMD_MOTION, enc(tq, -RS05_TORQUE_MAX, RS05_TORQUE_MAX), m->id, d) == HAL_OK;
}

/* 解析 RS05 反馈帧并刷新在线状态。 */
bool RS05_OnFrame(rs05_t *m, const can_frame_t *f)
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
    if (cmd != RS05_CMD_FEEDBACK || (uint8_t)f->id != RS05_HOST_ID || (uint8_t)(f->id >> 8) != m->id)
    {
        return false;
    }
    // 从 CAN ID 中解析电机故障码
    m->fault = (uint8_t)((f->id >> 16) & 0x3FU);
    // 解析单圈位置，并根据电机方向修正符号
    m->single_position = (float)m->direction * dec(rdbe(&f->data[0]), RS05_POSITION_MIN, RS05_POSITION_MAX);
    // 解析速度，并根据电机方向修正符号
    m->velocity = (float)m->direction * dec(rdbe(&f->data[2]), -RS05_VELOCITY_MAX, RS05_VELOCITY_MAX);
    // 解析力矩，并根据电机方向修正符号
    m->torque = (float)m->direction * dec(rdbe(&f->data[4]), -RS05_TORQUE_MAX, RS05_TORQUE_MAX);
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
            remainderf(m->single_position - remainderf(m->continuous_position, RS05_ANGLE_PERIOD), RS05_ANGLE_PERIOD);
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
        /* 掉线前若正在执行用户位置目标，恢复后保留原目标并重新起轨迹。 */
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

/* 读取 RS05 连续位置并转换为角度输出。 */
void RS05_GetPositionDegrees(const rs05_t *m, volatile float *position_degrees)
{
    if (position_degrees != NULL)
    {
        *position_degrees = (m == NULL) ? 0.0f : m->position * 57.2957795131f;
    }
}

/* 周期维护 RS05 零点保存和运动控制帧，不使用反馈看门狗。 */
void RS05_Service(rs05_t *m)
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
        (void)RS05_SendSetZero(m);
        // 收到有效反馈且当前位置接近零点时，清除设置请求
        if (m->feedback_valid && fabsf(m->single_position) <= RS05_ZERO_POSITION_TOLERANCE_RAD)
        {
            m->set_zero_position_request = 0U;
        }
        // 设置零点期间不执行后续控制流程
        return;
    }
    /* 不使用反馈超时看门狗；没有反馈时也继续按当前目标发送控制帧。 */
    if (m->recovery_trajectory && g_rs_recovery_motion_release == 0U)
    {
        return;
    }

    if (m->mode == RS_MODE_POSITION || m->mode == RS_MODE_SPEED || m->mode == RS_MODE_DAMPING || m->mode == RS_MODE_MIT)
    {
        if ((uint32_t)(now - m->last_control_tick) >= 5U)
        {
            m->last_control_tick = now;
            (void)RS05_Send(m);
        }
    }
}

void RS05_SetTargetMIT(rs05_t *motor, rs_mode_t mode, float tau_nm, float output_degrees)
{
    if (!motor || mode != RS_MODE_MIT || !isfinite(tau_nm) || !isfinite(output_degrees))
        return;
    if (motor->target_sink)
    {
        motor->target_sink(motor, (int)mode, tau_nm, output_degrees);
        return;
    }
    if (fabsf(output_degrees * RS05_DEG_TO_RAD) > RS05_POSITION_MAX || fabsf(tau_nm) > RS05_TORQUE_MAX)
        return;
    motor->mode = RS_MODE_MIT;
    motor->target = motor->direction * output_degrees * RS05_DEG_TO_RAD;
    motor->tau_ff_nm = tau_nm;
    motor->trajectory_pending = false;
    motor->trajectory_active = false;
    motor->startup_hold_active = true;
    motor->recovery_trajectory = false;
    PID_Reset(&motor->cascade_position_pid);
    PID_Reset(&motor->cascade_speed_pid);
}
