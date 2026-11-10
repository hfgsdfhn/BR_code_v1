/*
 * @file j4310.h
 * @brief 定义 J4310 的工作模式、反馈和控制接口。
 */

/* DM J4310 MIT 协议参数、状态结构和控制接口声明。 */
#ifndef J4310_H
#define J4310_H /* J4310 驱动头文件保护宏。 */

#include "bsp_can.h"
#include "pid.h"
#include "trapezoid_planner.h"
#include <stdbool.h>
#include <stdint.h>

/* 速度规划的重起步阈值与单周期最大积分步长。 */
#define J4310_SPEED_PLANNER_RESEED_SEC 0.1f
#define J4310_SPEED_PLANNER_MAX_DT_SEC 0.02f
#define J4310_ENABLE_COMMAND 0xFCU              /* J4310 MIT 协议使能命令字节。 */
#define J4310_DISABLE_COMMAND 0xFDU             /* J4310 MIT 协议失能命令字节。 */
#define J4310_DEG_TO_RAD 0.017453292519943295f  /* 位置目标由度转换为内部弧度。 */
#define J4310_SET_ZERO 0xFEU                    /* J4310 设置当前位置为零点的特殊命令码。 */
#define J4310_ZERO_POSITION_TOLERANCE_RAD 0.01f /* 允许保存零点的当前位置误差，单位：弧度。 */
#define J4310_PI 3.14159265359f                 /* 圆周率，用于角度换算。 */
#define J4310_TWO_PI 6.28318530718f             /* 二倍圆周率，用于角度回绕。 */
#define J4310_POSITION_MAX 12.5f                /* J4310 MIT 协议位置字段映射的关节位置绝对上限，单位：弧度。 */
#define J4310_SPEED_MAX 30.0f                   /* J4310 MIT 协议速度字段映射的关节速度绝对上限，单位：弧度每秒。 */
#define J4310_TORQUE_MAX 10.0f                  /* J4310 MIT 协议转矩字段映射的输出转矩绝对上限，单位：牛米。 */
#define J4310_KP_MAX 500.0f                     /* J4310 MIT 协议比例增益字段映射的最大值。 */
#define J4310_KD_MAX 5.0f                       /* J4310 MIT 协议微分增益字段映射的最大值。 */
#define J4310_STATE_ENABLED 0x01U               /* J4310 反馈中表示电机已使能的状态码。 */
#define J4310_STARTUP_HOLD_KP 30.0f             /* 上电当前位置保持使用的 MIT 比例增益。 */
#define J4310_STARTUP_HOLD_KD 0.95f             /* 上电当前位置保持使用的 MIT 微分增益。 */
#define J4310_STARTUP_ENABLE_RETRY_CYCLES 20U   /* 上电使能命令的控制周期重发间隔。 */
#define J4310_SPD_MAX_RAD_S J4310_SPEED_MAX     /* J4310 速度模式最大关节速度，单位：弧度每秒。 */
#define J4310_SPEED_ID_OFFSET 0x200U            /* 速度模式控制帧在节点号上叠加的偏移。 */
#define J4310_TRAJECTORY_PERIOD_MS 1U           /* J4310 五次轨迹的采样周期，单位：毫秒。 */
#define J4310_TRAJECTORY_EPSILON_RAD 0.0001f    /* J4310 将轨迹行程视为零时使用的位置容差。 */

typedef struct
{
    bool enabled;             /* 在线 MIT 增益调节是否启用。 */
    bool started;             /* 在线调节状态机是否已经启动。 */
    float base_kp;            /* 用户设定的基础比例增益。 */
    float base_kd;            /* 用户设定的基础微分增益。 */
    float applied_kp;         /* 当前实际使用的比例增益。 */
    float applied_kd;         /* 当前实际使用的微分增益。 */
    float previous_error;     /* 上一次位置误差。 */
    float previous_abs_error; /* 上一次位置误差绝对值。 */
    float convergence_rate;   /* 误差收敛速率。 */
} j4310_online_mit_state_t;

typedef enum
{
    J4310_MODE_STOP = 0U, /* 停止输出。 */
    J4310_MODE_DAMPING,   /* 速度阻尼，不锁定角度。 */
    J4310_MODE_SPEED,     /* 速度闭环，目标单位 rad/s。 */
    J4310_MODE_POSITION,  /* 位置外环嵌套速度内环，目标单位度。 */
    J4310_MODE_MIT        /* MIT：独立力矩前馈与输出轴角度。 */
} j4310_mode_t;           /* 表示 J4310 当前采用的控制模式。 */

typedef struct
{
    uint16_t age_cycles;       /* 距离最近一次有效反馈经过的控制周期数。 */
    float position_rad;        /* J4310 最近反馈的关节位置，单位：弧度。 */
    float speed_rad_s;         /* J4310 最近反馈的关节速度，单位：弧度每秒。 */
    float torque_nm;           /* J4310 最近反馈的输出转矩，单位：牛米。 */
    uint8_t state;             /* J4310 最近反馈的协议状态码。 */
    uint8_t mos_temperature;   /* J4310 功率管温度，单位：摄氏度。 */
    uint8_t rotor_temperature; /* J4310 转子温度，单位：摄氏度。 */
    bool received;             /* 是否已经收到 J4310 的有效反馈。 */
} j4310_feedback_t;            /* 保存 J4310 最近一次有效反馈的状态和测量值。 */

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid;
    uint32_t cascade_tick;
    bool cascade_configured; /* Optional sole bus-owner callback. */

    uint8_t motor_id;                 /* J4310 在电机侧使用的协议节点编号。 */
    uint16_t master_id;               /* J4310 反馈帧发送到的主控节点编号。 */
    int direction;                    /* J4310 电机坐标系到机构坐标系的方向系数，取 1 或 -1。 */
    j4310_mode_t mode;                /* J4310 当前采用的控制模式。 */
    float target_position_rad;        /* J4310 MIT 控制的目标关节位置，单位：弧度。 */
    float target_speed_rad_s;         /* J4310 MIT 控制的目标关节速度，单位：弧度每秒。 */
    float reference_position_rad;     /* J4310 五次轨迹本周期的参考位置，单位：弧度。 */
    float reference_speed_rad_s;      /* J4310 五次轨迹本周期的参考速度，单位：弧度每秒。 */
    float trajectory_start_rad;       /* J4310 当前五次轨迹的起始位置，单位：弧度。 */
    float trajectory_k0;              /* 归一化初速度 = 起始速度/平均速度；0 表示静止起步。 */
    uint32_t trajectory_started_at;   /* J4310 当前轨迹开始执行的系统毫秒时刻。 */
    uint32_t trajectory_duration_ms;  /* J4310 当前轨迹的总时长，单位：毫秒。 */
    float target_torque_nm;           /* J4310 MIT 控制叠加的前馈转矩，单位：牛米。 */
    float kp;                         /* J4310 MIT 位置误差的比例增益。 */
    float kd;                         /* J4310 MIT 速度误差的微分增益。 */
    j4310_feedback_t feedback;        /* J4310 最近一次有效反馈。 */
    float last_raw_position_rad;      /* J4310 上一帧按机构方向换算后的协议位置，单位：弧度。 */
    float position_offset_rad;        /* 连续关节位置换算回当前协议位置分支时使用的偏移，单位：弧度。 */
    uint8_t startup_enable_countdown; /* 距离下一次上电使能重发剩余的控制周期数。 */
    bool position_tracking_valid;     /* 是否已经建立 J4310 连续位置跟踪。 */
    bool offline_latched;             /* 是否已经检测到反馈掉线。 */
    bool fault_active;                /* 当前反馈是否为需要失能的 J4310 故障状态。 */
    bool fault_disable_sent;          /* 当前故障期间是否已经发送过失能帧。 */
    bool fault_recovery_pending;      /* 故障清除后是否仍在等待重新使能确认。 */
    bool trajectory_pending;          /* 新位置目标是否等待以实测位置建立轨迹。 */
    bool trajectory_active;           /* J4310 五次位置轨迹当前是否正在运行。 */
    trapezoid_t speed_planner;        /* 速度模式的 T 型规划状态。 */
    uint32_t speed_planner_tick;      /* 上次速度规划时刻，单位 ms。 */
    bool speed_planner_valid;         /* 速度规划状态是否已有初值。 */
    bool startup_hold_pending;        /* 是否仍在等待有效反馈以锁定上电角度。 */
    bool startup_hold_active;         /* 用户位置控制是否持续输出；首帧确认后关闭。 */
    bool recovery_trajectory;         /* 当前轨迹是否为自动回零轨迹。 */
    volatile uint8_t set_zero_position_request;
} j4310_t; /* 保存 J4310 的协议地址、控制目标、增益和反馈。 */

extern volatile float j4310_trajectory_max_speed_rad_s;         /* 全部 J4310 共用的五次轨迹最大速度。 */
extern volatile float j4310_trajectory_max_acceleration_rad_s2; /* 全部 J4310 共用的五次轨迹最大加速度。 */
extern volatile float j4310_trajectory_time_s;                  /* 0=按限幅反解时长；>0=按该时长(秒)正解峰值。 */
/* 速度模式 T 型规划的加速度上限，单位：弧度/秒^2。 */
extern volatile float j4310_speed_max_acceleration_rad_s2;
/* MIT 模式转矩前馈占位参数，单位：牛米；当前默认 0，标定后直接改这个全局。 */
extern volatile float j4310_tau_ff_nm;

/* 功能：初始化 J4310 地址和控制模式。 */
void J4310_Init(j4310_t *motor /* 待初始化的 J4310 控制对象。 */,
                uint8_t motor_id /* 初始化时绑定的 J4310 电机侧节点编号。 */,
                uint16_t master_id /* 初始化时绑定的 J4310 主控侧节点编号。 */,
                int direction /* 电机坐标系到机构坐标系的方向系数，取 1 或 -1。 */,
                j4310_mode_t mode /* J4310 初始化后采用的控制模式。 */);

/* 功能：切换 J4310 控制模式。 */
void J4310_SetMode(j4310_t *motor /* 待切换模式的 J4310 控制对象。 */, j4310_mode_t mode /* J4310 新的控制模式。 */);

/* 一次设置 J4310 控制模式及对应目标值。 */
void J4310_SetTargetMode(j4310_t *motor /* 待设置目标的 J4310 对象。 */, j4310_mode_t mode /* 新的控制模式。 */,
                         float target /* 位置模式单位度，速度/阻尼模式单位弧度/秒。 */,
                         float speed_rad_s /* 目标速度，单位：弧度/秒。 */, float kp /* MIT 比例增益。 */,
                         float kd /* MIT 微分增益。 */, float torque_nm /* 前馈转矩，单位：牛米。 */);

/* 功能：停止 J4310 轨迹和力矩输出。 */
bool J4310_Stop(j4310_t *motor /* 待停止的 J4310 控制对象。 */);

/* 功能：让 J4310 重新进入周期使能流程。 */
void J4310_RequestEnable(j4310_t *motor /* 需要重新请求使能的 J4310 控制对象。 */);

/* 功能：设置 J4310 的位置、速度、增益和前馈转矩目标。 */
void J4310_SetTargetBasic(j4310_t *motor, j4310_mode_t mode, float target);
void J4310_SetMitTarget(j4310_t *motor /* 待设置 MIT 目标的 J4310 控制对象。 */,
                        float position_degrees /* J4310 的目标关节位置，单位：度。 */,
                        float speed_rad_s /* J4310 的目标关节速度，单位：弧度每秒。 */,
                        float kp /* 本次目标设置采用的 MIT 位置比例增益。 */,
                        float kd /* 本次目标设置采用的 MIT 速度微分增益。 */,
                        float torque_nm /* J4310 需要叠加的前馈转矩，单位：牛米。 */);

/* 功能：以度为单位设置 J4310 关节位置目标；无返回值表示平滑轨迹已等待建立。 */
void J4310_SetPositionDegrees(j4310_t *motor /* 待设置位置目标的 J4310 对象。 */,
                              float degrees /* J4310 关节目标角度，单位：度。 */,
                              float kp /* J4310 MIT 位置误差的比例增益。 */,
                              float kd /* J4310 MIT 速度误差的微分增益。 */,
                              float torque_nm /* J4310 需要叠加的前馈转矩，单位：牛米。 */);

/* 功能：解析一帧 J4310 反馈。 */
bool J4310_OnFrame(j4310_t *motor /* 待更新反馈的 J4310 控制对象。 */,
                   const can_frame_t *frame /* 待解析的 J4310 反馈帧。 */);
void J4310_GetPositionDegrees(const j4310_t *motor /* 待读取位置的 J4310 控制对象。 */,
                              volatile float *position_degrees /* 用于写出关节位置，单位：度。 */);

/* 功能：构造 J4310 使能指令。 */
void J4310_BuildEnable(const j4310_t *motor /* 提供使能命令目标地址的 J4310 控制对象。 */,
                       can_frame_t *frame /* 用于写出 J4310 使能命令的 CAN 帧。 */);

/* 功能：构造 J4310 失能指令。 */
void J4310_BuildDisable(const j4310_t *motor /* 提供失能命令目标地址的 J4310 控制对象。 */,
                        can_frame_t *frame /* 用于写出 J4310 失能命令的 CAN 帧。 */);

/* 功能：构造 J4310 MIT 控制指令。 */
void J4310_BuildMit(const j4310_t *motor /* 提供协议地址和模式的 J4310 控制对象。 */,
                    float position_rad /* 待编码的 J4310 目标关节位置，单位：弧度。 */,
                    float speed_rad_s /* 待编码的 J4310 目标关节速度，单位：弧度每秒。 */,
                    float kp /* 待编码的 J4310 MIT 比例增益。 */, float kd /* 待编码的 J4310 MIT 微分增益。 */,
                    float torque_nm /* 待编码的 J4310 前馈转矩，单位：牛米。 */,
                    can_frame_t *frame /* 用于写出 J4310 MIT 命令的 CAN 帧。 */);

/* 功能：生成 J4310 上电使能或正常 MIT 控制帧；返回 true 表示本周期需要发送生成的帧。 */
bool J4310_BuildControlFrame(j4310_t *motor /* 参与上电保持和正常控制的 J4310 对象。 */,
                             float additional_torque_nm /* 需要叠加的补偿转矩，单位：牛米。 */,
                             uint32_t now /* 用于采样五次位置轨迹的当前毫秒时刻。 */,
                             can_frame_t *frame /* 用于写出本周期 J4310 命令的 CAN 帧。 */);

/* 普通模式三个参数；MIT 四个参数：对象、MIT、tau(Nm)、输出轴角度(deg)。
 * 编译期按参数数量分派，不使用不安全的运行期可变参数。 */
void J4310_SetTargetMIT(j4310_t *motor, j4310_mode_t mode, float tau_nm, float output_degrees);
#define J4310_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define J4310_SetTarget(...) J4310_TARGET_SELECT(__VA_ARGS__, J4310_SetTargetMIT, J4310_SetTargetBasic)(__VA_ARGS__)

/* 独立驱动 POSITION 双环参数；托管机械臂参数见 arm_config.h。 */
#define J4310_POSITION_PID_KP 4.0f
#define J4310_POSITION_PID_KI 0.0f
#define J4310_POSITION_PID_KD 0.0f
#define J4310_POSITION_PID_I_LIMIT 0.0f
#define J4310_POSITION_SPEED_LIMIT 0.5f
#define J4310_SPEED_PID_KP 1.0f
#define J4310_SPEED_PID_KI 0.1f
#define J4310_SPEED_PID_KD 0.0f
#define J4310_SPEED_PID_I_LIMIT 1.0f
#define J4310_CASCADE_PERIOD_S 0.002f
#define J4310_MIT_POSITION_KP 20.0f
#define J4310_MIT_VELOCITY_KD 1.0f

#endif
