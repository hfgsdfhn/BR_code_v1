/*
 * @file m3508.h
 * @brief 定义 M3508 电机反馈、控制数据和接口。
 */

/* DJI M3508 参数、状态结构和控制接口声明。 */
#ifndef M3508_H
#define M3508_H /* M3508 驱动头文件保护宏。 */

#include "bsp_can.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#define M3508_ENABLE 1U                          /* M3508 使能特殊命令码。 */
#define M3508_DISABLE 0U                         /* M3508 失能特殊命令码。 */
#define M3508_DEG_TO_RAD 0.017453292519943295f   /* 位置模式输入角度转换为内部弧度。 */
#define M3508_MIN_TRAJECTORY_S 0.01f             /* 位置变化轨迹的最小时长，单位：秒。 */
#define M3508_TRAJECTORY_EPSILON 0.000001f       /* 判断轨迹是否完成的归一化误差。 */
#define M3508_FEEDBACK_TIMEOUT_CYCLES 250U       /* 2 ms 周期下超过 500 ms 才标记离线。 */
#define M3508_MOTOR_COUNT 8U                     /* 一组 DJI 电流控制帧可寻址的 M3508 电机总数。 */
#define M3508_ENCODER_COUNT 8192.0f              /* M3508 转子编码器每圈的计数值。 */
#define M3508_REDUCTION_RATIO (3591.0f / 187.0f) /* M3508 转子到机械臂输出轴的减速比。 */
#define M3508_CURRENT_SCALE (20.0f / 16384.0f)   /* M3508 协议电流原始值换算为安培时的比例。 */
#define M3508_CURRENT_RAW_LIMIT 16384.0f         /* M3508 电流命令允许使用的协议原始值绝对上限。 */
#define M3508_CUR_MAX_A 20.0f                    /* M3508 电流模式最大电流，单位：安培。 */
#define M3508_SPD_MAX_RAD_S 31.416f              /* M3508 速度模式最大输出轴速度，单位：弧度每秒。 */
#define M3508_ZERO_STABLE_FRAMES 1U              /* 首帧有效反馈立即建立软件零点。 */
#define M3508_ZERO_MAX_STEP_COUNTS 128           /* 建立软件零点时相邻编码器反馈允许的最大变化量。 */
#define M3508_ZERO_MAX_SPEED_RPM 30              /* 建立软件零点时允许的最大转子转速，单位：转每分。 */
#define RPM_TO_RAD_S 0.10471975512f              /* 将 M3508 转子速度由转每分换算为弧度每秒的系数。 */
#define TWO_PI 6.28318530718f                    /* 将 M3508 编码器圈数换算为转子弧度时使用的二倍圆周率。 */

typedef enum
{
    M3508_MODE_STOP = 0U, /* 停止输出。 */
    M3508_MODE_DAMPING,   /* 速度阻尼，不锁定角度。 */
    M3508_MODE_SPEED,     /* 速度闭环，目标单位 rad/s。 */
    M3508_MODE_POSITION,  /* 位置外环嵌套速度内环，目标单位度。 */
    M3508_MODE_CURRENT    /* DJI 原生电流模式；不支持 MIT。 */
} m3508_mode_t;           /* 表示 M3508 当前采用的控制模式。 */

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target,
                        float position_deg); /* Optional sole bus-owner callback. */

    FDCAN_HandleTypeDef *hcan;           /* 绑定的 FDCAN 外设。 */
    uint16_t feedback_age_cycles;        /* 距离最近一次有效反馈经过的控制周期数。 */
    uint8_t id;                          /* M3508 的 DJI 协议节点编号，范围为 1 至 8。 */
    int direction;                       /* M3508 电机坐标系到机械臂坐标系的方向系数，取 1 或 -1。 */
    uint16_t encoder;                    /* M3508 最近反馈的单圈转子编码器原始值。 */
    uint16_t last_encoder;               /* M3508 上一帧反馈的单圈转子编码器原始值。 */
    int32_t total_encoder;               /* 展开编码器过零后得到的累计转子计数。 */
    int16_t speed_rpm;                   /* M3508 最近反馈的转子速度，单位：转每分。 */
    int16_t current_raw;                 /* M3508 按机构方向换算后的反馈转矩电流原始值。 */
    float position_rad;                  /* M3508 减速后机械臂输出轴的累计位置，单位：弧度。 */
    float speed_rad_s;                   /* M3508 减速后机械臂输出轴的速度，单位：弧度每秒。 */
    float target_current_a;              /* 电流模式下的 M3508 目标电流，单位：安培。 */
    float target_position_rad;           /* 位置模式下的机械臂 M3508 输出轴目标位置，单位：弧度。 */
    float target_speed_rad_s;            /* 速度或阻尼模式下的机械臂 M3508 输出轴目标速度，单位：弧度每秒。 */
    float reference_position_rad;        /* 五次位置轨迹本周期的参考位置，单位：弧度。 */
    float reference_speed_rad_s;         /* 位置轨迹或速度斜坡本周期的参考速度，单位：弧度每秒。 */
    float reference_acceleration_rad_s2; /* 本周期轨迹参考加速度，单位：弧度每二次方秒。 */
    float trajectory_start_rad;          /* 当前五次位置轨迹的起始位置，单位：弧度。 */
    float trajectory_delta_rad;          /* 当前五次位置轨迹的总位移，单位：弧度。 */
    float trajectory_k0;                 /* 归一化初速度 = 起始速度/平均速度；0 表示静止起动。 */
    float trajectory_duration_s;         /* 当前五次位置轨迹的总时长，单位：秒。 */
    float trajectory_elapsed_s;          /* 当前五次位置轨迹已经运行的时间，单位：秒。 */
    pid_t position_pid;                  /* M3508 输出轴位置外环 PID 控制器。 */
    pid_t speed_pid;                     /* M3508 输出轴速度内环 PID 控制器。 */
    pid_t damping_pid;                   /* 使用速度反馈生成阻尼电流的 PID 控制器。 */
    m3508_mode_t mode;                   /* M3508 当前采用的控制模式。 */
    uint8_t zero_stable_frames;          /* 上电软件零点已累计的连续稳定反馈帧数。 */
    bool received;                       /* 是否已经收到该 M3508 的有效反馈。 */
    bool feedback_online;                /* 最近是否已经收到可用于闭环的反馈。 */
    bool speed_reference_valid;          /* 速度斜坡参考值是否已经由反馈初始化。 */
    bool trajectory_pending;             /* 新位置目标是否等待以实测位置建立轨迹。 */
    bool trajectory_active;              /* 当前是否正在运行五次位置轨迹。 */
    bool startup_hold_active;            /* 首帧软件零点保持是否有效；位置命令保留，其他模式清除。 */
} m3508_t;                               /* 保存一台 M3508 的地址、反馈、控制目标和 PID 状态。 */

extern volatile float m3508_trajectory_max_speed_rad_s;         /* 全部 M3508 共用的五次轨迹最大速度。 */
extern volatile float m3508_trajectory_max_acceleration_rad_s2; /* 全部 M3508 共用的五次轨迹最大加速度。 */
extern volatile float m3508_trajectory_time_s;                  /* 0=按限幅反解时长；>0=按该时长(秒)正解峰值。 */

/* 功能：使用 M3508 参数初始化电机。 */
HAL_StatusTypeDef M3508_Init(m3508_t *motor /* 待初始化的 M3508 控制对象。 */,
                             FDCAN_HandleTypeDef *hcan /* FDCAN 外设句柄。 */,
                             uint8_t id /* 初始化时绑定的 M3508 节点编号，范围为 1 至 8。 */,
                             int direction /* 电机坐标系到机械臂坐标系的方向系数，取 1 或 -1。 */);

/* 功能：解析一帧 M3508 电机反馈。 */
bool M3508_OnFrame(m3508_t *motor /* 待更新反馈的 M3508 控制对象。 */,
                   const can_frame_t *frame /* 待解析的 DJI 电机反馈帧。 */);

/* 功能：设置 M3508 电机控制模式和目标值。 */
void M3508_SetTarget(m3508_t *motor /* 待设置目标的 M3508 控制对象。 */,
                     m3508_mode_t mode /* 本次目标采用的 M3508 控制模式。 */,
                     float target /* 电流单位 A、速度单位 rad/s、位置单位度。 */);

/* 功能：停止 M3508 输出并立即发送一次零电流指令。 */
bool M3508_Stop(m3508_t *motor /* 待停止的 M3508 控制对象。 */);

/* 功能：设置两台 M3508 的共同输出轴角度目标；无返回值表示两台电机的位置轨迹已等待建立。 */
void M3508_SetPairPositionDegrees(m3508_t motors[2] /* 待同步控制的两台 M3508 对象。 */,
                                  float degrees /* M3508 输出轴共同目标角度，单位：度。 */);

/* 功能：读取反馈并计算本周期电流指令。 */
int16_t M3508_Calc(m3508_t *motor /* 参与本周期闭环计算的 M3508 控制对象。 */,
                   float dt_s /* 相邻两次控制计算的时间间隔，单位：秒。 */);

/* 功能：将四台 M3508 电机电流指令组成一帧 CAN 数据。 */
void M3508_BuildGroupFrame(uint8_t first_id /* 本帧第一个电流通道对应的 M3508 节点编号。 */,
                           const int16_t current[4] /* 四台连续编号 M3508 的协议电流原始值。 */,
                           can_frame_t *frame /* 用于写出 DJI 分组电流命令的 CAN 帧。 */);

/* 功能：将 M3508 标记为反馈离线并复位控制器；用途：恢复时保留 MCU 初次上电建立的软件零点。 */
void M3508_ResetFeedback(m3508_t *motor /* 需要等待反馈恢复的 M3508 控制对象。 */);
void M3508_Service(m3508_t *motor /* 周期维持未切换到其他模式的上电软件零点保持。 */,
                   float dt_s /* 本次控制周期，单位：秒。 */);
/* 读取 M3508 输出轴连续位置并转换为角度。 */
void M3508_GetPositionDegrees(const m3508_t *motor /* 待读取位置的 M3508 控制对象。 */,
                              volatile float *position_degrees /* 用于写出输出轴位置，单位：度。 */);

#ifndef M3508_BUILD_SPECIAL_DECLARED
#define M3508_BUILD_SPECIAL_DECLARED /* 防止特殊命令接口重复声明。 */

void M3508_BuildSpecial(m3508_t *motor /* 提供目标地址的 M3508 对象。 */, uint8_t command /* 特殊命令码。 */);
#endif

/* Update state only; shared-bus owners aggregate and send current frames. */
void M3508_SetTargetDeferred(m3508_t *motor, m3508_mode_t mode, float target);

/* Auxiliary position/speed PID tuning. */
#define M3508_POSITION_KP 10.4720f
#define M3508_POSITION_KI 0.0f
#define M3508_POSITION_KD 0.0f
#define M3508_POSITION_INTEGRAL_LIMIT 0.0f
#define M3508_SPEED_KP 1.3988f
#define M3508_SPEED_KI 0.9325f
#define M3508_SPEED_KD 0.0f
#define M3508_SPEED_OUTPUT_LIMIT 3.0f
#define M3508_SPEED_INTEGRAL_LIMIT 6.0f
#define M3508_POSITION_I_FULL 0.01f
#define M3508_POSITION_I_SEPARATION 0.25f
#define M3508_SPEED_I_FULL 0.314159f
#define M3508_SPEED_I_SEPARATION 4.712389f

#endif
