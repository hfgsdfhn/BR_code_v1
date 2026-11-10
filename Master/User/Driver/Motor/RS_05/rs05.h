/*
 * @file rs05.h
 * @brief RS05 电机驱动的数据类型、协议范围和公开接口。
 */

/* RS05 协议范围、运行模式和控制接口声明。 */
#ifndef RS05_H
#define RS05_H /* RS05 驱动头文件保护宏。 */

#include "bsp_can.h"
#include "pid.h"
#include "trapezoid_planner.h"
#include <stdbool.h>
#include <stdint.h>

#define RS05_SET_ZERO 6U                       /* RS05 设置当前位置为零点的命令码。 */
#define RS05_ZERO_POSITION_TOLERANCE_RAD 0.01f /* 允许保存零点的当前位置误差，单位：弧度。 */

#define RS05_ENABLE 3U       /* RS05 使能特殊命令码。 */
#define RS05_FALSE 4U        /* RS05 失能特殊命令码。 */
#define RS05_CMD_MOTION 1U   /* RS05 运动控制命令码。 */
#define RS05_CMD_FEEDBACK 2U /* RS05 反馈帧命令码。 */

/* 通信类型 18：单个参数写入（掉电丢失）。 */
#define RS05_CMD_WRITE_PARAM 0x12U

/* 运行模式取值：0 = 运控（MIT），2 = 速度模式。 */
#define RS05_RUN_MODE_MIT 0U
#define RS05_RUN_MODE_SPEED 2U

/* 参数索引，见手册参数表。 */
#define RS05_IDX_RUN_MODE 0x7005U
#define RS05_IDX_SPD_REF 0x700AU
#define RS05_IDX_LIMIT_CUR 0x7018U
#define RS05_IDX_ACC_RAD 0x7022U
#define RS05_HOST_ID 0xFDU /* RS05 主控端 CAN 标识。 */
#ifndef RS_MODE_T_DEFINED
#define RS_MODE_T_DEFINED /* 防止 RS 模式枚举被重复定义。 */

typedef enum
{
    RS_MODE_STOP = 0U, /* 停止输出。 */
    RS_MODE_DAMPING,   /* 速度阻尼，不锁定角度。 */
    RS_MODE_SPEED,     /* 速度闭环，目标单位 rad/s。 */
    RS_MODE_POSITION,  /* 位置外环嵌套速度内环，目标单位度。 */
    RS_MODE_MIT        /* MIT：独立力矩前馈与输出轴角度。 */
} rs_mode_t;
#endif

/* 速度规划的重起步阈值：控制周期超过该值说明速度环没有连续运行，改用实测速度重新起步。 */
#define RS05_SPEED_PLANNER_RESEED_SEC 0.1f

/* 单周期最大积分步长，防止一次异常长间隔让速度跳变过大。 */
#define RS05_SPEED_PLANNER_MAX_DT_SEC 0.02f
#define RS05_ANGLE_PERIOD 6.28318530718f      /* RS 电机角度回绕周期，单位：弧度。 */
#define RS05_DEG_TO_RAD 0.017453292519943295f /* RS05 角度由度转换为弧度的系数。 */

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid;
    uint32_t cascade_tick;
    bool cascade_configured; /* Optional sole bus-owner callback. */

    FDCAN_HandleTypeDef *hcan;       /* 绑定的 FDCAN 外设。 */
    uint8_t id;                      /* 电机节点 ID。 */
    int direction;                   /* 电机坐标到机构坐标的方向系数，取 1 或 -1。 */
    rs_mode_t mode;                  /* 当前控制模式。 */
    float target;                    /* 当前模式下的目标值。 */
    float hold_position;             /* 速度模式下保持的反馈位置。 */
    float reference_position;        /* 五次位置轨迹本周期的参考位置，单位 rad。 */
    float reference_velocity;        /* 五次位置轨迹本周期的参考速度，单位 rad/s。 */
    float trajectory_start_position; /* 当前五次轨迹的起始位置，单位 rad。 */
    float trajectory_delta_position; /* 当前五次轨迹的总位移，单位 rad。 */
    float trajectory_k0;             /* 归一化初速度 = v0*T/位移；0 表示静止起动。 */
    uint32_t trajectory_start_tick;  /* 当前五次轨迹的起始毫秒时刻。 */
    uint32_t trajectory_duration_ms; /* 当前五次轨迹的总时长，单位 ms。 */
    volatile uint32_t last_feedback_tick;
    volatile float position; /* 输出轴位置，单位 rad。 */
    volatile float single_position;
    volatile float velocity;                 /* 输出轴速度，单位 rad/s。 */
    volatile float torque;                   /* 反馈转矩，单位 N·m。 */
    volatile uint8_t fault;                  /* 驱动器故障码。 */
    volatile bool feedback_valid;            /* 是否已经收到有效反馈。 */
    volatile bool feedback_recovery_pending; /* 恢复首帧只重建目标，不发送运动帧。 */
    volatile bool enabled;                   /* 最近一次使能命令是否成功。 */
    float continuous_position;
    bool continuous_valid;
    bool angle_store_pending;
    volatile bool offline_latched;
    volatile bool ever_feedback_received;
    volatile uint32_t reconnect_retry_tick; /* 最近一次重连使能命令的时刻。 */
    volatile uint32_t last_poll_tick;
    volatile uint32_t last_control_tick;
    bool trajectory_pending;                /* 新位置目标是否等待以实测位置建立轨迹。 */
    bool trajectory_active;                 /* 当前是否正在执行五次位置轨迹。 */
    volatile bool startup_hold_active;      /* 位置控制是否需要持续输出；首帧不激活，用户位置命令激活。 */
    volatile bool startup_hold_initialized; /* 一次性上电位置保持是否已经建立。 */
    volatile bool recovery_trajectory;      /* 当前轨迹是否为自动回零轨迹。 */
    volatile bool recovery_sync_candidate;  /* 是否需要与另一台 RS 电机同步恢复。 */
    volatile uint8_t set_zero_position_request;
    trapezoid_t speed_planner;   /* 速度模式的 T 型规划状态。 */
    uint32_t speed_planner_tick; /* 上次速度规划时刻，单位 ms。 */
    bool speed_planner_valid;    /* 速度规划状态是否已有初值。 */
    float tau_ff_nm;             /* MIT 转矩前馈占位参数，单位：牛米，默认 0。 */
    uint8_t run_mode;            /* 电机当前运行模式：0 = MIT，2 = 速度。 */
} rs05_t;

extern volatile float rs05_trajectory_max_speed_rad_s;         /* 全部 RS05 共用的五次轨迹最大速度。 */
extern volatile float rs05_trajectory_max_acceleration_rad_s2; /* 全部 RS05 共用的五次轨迹最大加速度。 */
extern volatile float rs05_trajectory_time_s;                  /* 0=按限幅反解时长；>0=按该时长(秒)正解峰值。 */
extern volatile float rs05_speed_max_acceleration_rad_s2;      /* 速度模式 T 型规划的加速度上限，单位：弧度/秒^2。 */
extern volatile float rs05_speed_limit_current_a;              /* 速度模式电流限制，单位：安培。 */
extern volatile float rs05_speed_accel_rad_s2;                 /* 速度模式电机内部加速度参数，单位：弧度/秒^2。 */
/* 两种 RS 电机型号共用此基于反馈的恢复速度斜坡。 */
extern volatile float rs_recovery_max_speed_rad_s;
extern volatile float rs_recovery_max_acceleration_rad_s2;

float RS05_RecoveryVelocity(float error_rad, float previous_rad_s, float actual_rad_s, float dt_s);
#define RS05_POSITION_MIN (-12.57f)             /* 位置命令下限，单位：弧度。 */
#define RS05_POSITION_MAX (12.57f)              /* 位置命令上限，单位：弧度。 */
#define RS05_VELOCITY_MAX (50.0f)               /* 速度命令上限，单位：弧度/秒。 */
#define RS05_SPD_MAX (RS05_VELOCITY_MAX)        /* RS05 速度模式最大速度，单位：弧度/秒。 */
#define RS05_TORQUE_MAX (5.5f)                  /* 转矩命令上限，单位：牛米。 */
#define RS05_KP_MIN (0.0f)                      /* MIT 位置比例增益编码下限。 */
#define RS05_KP_MAX (500.0f)                    /* MIT 位置比例增益编码上限。 */
#define RS05_KD_MIN (0.0f)                      /* MIT 速度微分增益编码下限。 */
#define RS05_KD_MAX (5.0f)                      /* MIT 速度微分增益编码上限。 */
#define RS05_TRAJECTORY_PERIOD_MS (2U)          /* 五次轨迹更新周期，单位：毫秒。 */
#define RS05_TRAJECTORY_EPSILON_RAD (0.000001f) /* 判断位置目标是否变化的容差。 */

HAL_StatusTypeDef RS05_Init(rs05_t *motor /* 待初始化的 RS05 对象。 */,
                            FDCAN_HandleTypeDef *hcan /* 绑定的 FDCAN 外设。 */, uint8_t id /* RS05 节点编号。 */,
                            int direction /* 坐标方向系数，取 1 或 -1。 */);
bool RS05_BuildSpecial(rs05_t *motor /* 提供目标地址的 RS05 对象。 */, uint8_t command /* 特殊命令码。 */);
void RS05_SetTargetBasic(rs05_t *motor /* 待设置目标的 RS05 对象。 */, rs_mode_t mode /* 新的控制模式。 */,
                         float target /* 位置单位度，速度单位弧度/秒。 */);
bool RS05_Send(rs05_t *motor /* 待发送目标的 RS05 对象。 */);
bool RS05_OnFrame(rs05_t *motor /* 待更新反馈的 RS05 对象。 */, const can_frame_t *frame /* 待解析的 RS05 反馈帧。 */);
bool RS05_Stop(rs05_t *motor /* 待停止的 RS05 对象。 */);
/* 周期维护 RS05 的反馈、重连和运动控制帧。 */
void RS05_Service(rs05_t *motor /* 周期维护 RS05 的反馈、重连和运动控制。 */);
void RS05_GetPositionDegrees(const rs05_t *motor /* 待读取位置的 RS05 对象。 */,
                             volatile float *position_degrees /* 用于写出输出轴位置，单位：度。 */);

/* 普通模式三个参数；MIT 四个参数：对象、MIT、tau(Nm)、输出轴角度(deg)。
 * 编译期按参数数量分派，不使用不安全的运行期可变参数。 */
void RS05_SetTargetMIT(rs05_t *motor, rs_mode_t mode, float tau_nm, float output_degrees);
#define RS05_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define RS05_SetTarget(...) RS05_TARGET_SELECT(__VA_ARGS__, RS05_SetTargetMIT, RS05_SetTargetBasic)(__VA_ARGS__)

/* 独立驱动 POSITION 双环参数；托管机械臂参数见 arm_config.h。 */
#define RS05_POSITION_PID_KP 4.0f
#define RS05_POSITION_PID_KI 0.0f
#define RS05_POSITION_PID_KD 0.0f
#define RS05_POSITION_PID_I_LIMIT 0.0f
#define RS05_POSITION_SPEED_LIMIT 0.5f
#define RS05_SPEED_PID_KP 1.0f
#define RS05_SPEED_PID_KI 0.1f
#define RS05_SPEED_PID_KD 0.0f
#define RS05_SPEED_PID_I_LIMIT 1.0f
#define RS05_CASCADE_PERIOD_S 0.002f
#define RS05_MIT_POSITION_KP 20.0f
#define RS05_MIT_VELOCITY_KD 1.0f

#endif
