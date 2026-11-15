/*
 * @file rs05.h
 * @brief rs05模块的用户接口与实现。
 */

#ifndef RS05_H
#define RS05_H

#include "bsp_can.h"
#include "pid.h"
#include "trapezoid_planner.h"
#include <stdbool.h>
#include <stdint.h>

#define RS05_SET_ZERO 6U
#define RS05_ZERO_POSITION_TOLERANCE_RAD 0.01f

#define RS05_ENABLE 3U
#define RS05_FALSE 4U
#define RS05_CMD_MOTION 1U
#define RS05_CMD_FEEDBACK 2U

#define RS05_CMD_WRITE_PARAM 0x12U

#define RS05_RUN_MODE_MIT 0U
#define RS05_RUN_MODE_SPEED 2U

#define RS05_IDX_RUN_MODE 0x7005U
#define RS05_IDX_SPD_REF 0x700AU
#define RS05_IDX_LIMIT_CUR 0x7018U
#define RS05_IDX_ACC_RAD 0x7022U
#define RS05_HOST_ID 0xFDU
#ifndef RS_MODE_T_DEFINED
#define RS_MODE_T_DEFINED

typedef enum
{
    RS_MODE_STOP = 0U, /* 当前工作模式或状态。 */
    RS_MODE_DAMPING, /* 当前工作模式或状态。 */
    RS_MODE_SPEED, /* 当前工作模式或状态。 */
    RS_MODE_POSITION, /* 当前工作模式或状态。 */
    RS_MODE_MIT /* 当前工作模式或状态。 */
} rs_mode_t;
#endif

#define RS05_SPEED_PLANNER_RESEED_SEC 0.1f

#define RS05_SPEED_PLANNER_MAX_DT_SEC 0.02f
#define RS05_ANGLE_PERIOD 6.28318530718f
#define RS05_DEG_TO_RAD 0.017453292519943295f

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid;
    uint32_t cascade_tick;
    bool cascade_configured;
    float mit_kp, mit_kd;

    FDCAN_HandleTypeDef *hcan; /* 对应外设句柄。 */
    uint8_t id;
    int direction;
    rs_mode_t mode; /* 当前工作模式或状态。 */
    float target;
    float hold_position;
    float reference_position;
    float reference_velocity;
    float trajectory_time_s;
    float trajectory_start_position;
    float trajectory_delta_position;
    float trajectory_k0;
    uint32_t trajectory_start_tick;
    uint32_t trajectory_duration_ms;
    volatile uint32_t last_feedback_tick;
    volatile float position;
    volatile float single_position;
    volatile float velocity;
    volatile float torque;
    volatile uint8_t fault;
    volatile bool feedback_valid;
    volatile bool feedback_recovery_pending;
    volatile bool enabled;
    volatile uint8_t feedback_state;
    float continuous_position;
    bool continuous_valid;
    bool angle_store_pending;
    volatile bool offline_latched;
    volatile bool ever_feedback_received;
    volatile uint32_t reconnect_retry_tick;
    volatile uint32_t last_poll_tick;
    volatile uint32_t last_control_tick;
    bool trajectory_pending;
    bool trajectory_active;
    volatile bool startup_hold_active;
    volatile bool startup_hold_initialized;
    volatile bool recovery_trajectory;
    volatile bool recovery_sync_candidate;
    volatile uint8_t set_zero_position_request;
    trapezoid_t speed_planner;
    uint32_t speed_planner_tick;
    bool speed_planner_valid;
    float tau_ff_nm;
    uint8_t run_mode; /* 当前工作模式或状态。 */
} rs05_t;

extern volatile float rs05_trajectory_max_speed_rad_s;
extern volatile float rs05_trajectory_max_acceleration_rad_s2;
extern volatile float rs05_trajectory_time_s;
extern volatile float rs05_speed_max_acceleration_rad_s2;
extern volatile float rs05_speed_limit_current_a;
extern volatile float rs05_speed_accel_rad_s2;

extern volatile float rs_recovery_max_speed_rad_s;
extern volatile float rs_recovery_max_acceleration_rad_s2;

/* 功能：处理RS05_RecoveryVelocity对应的输入并更新相关状态。 */
float RS05_RecoveryVelocity(float error_rad, float previous_rad_s, float actual_rad_s, float dt_s);
#define RS05_POSITION_MIN (-12.57f)
#define RS05_POSITION_MAX (12.57f)
#define RS05_VELOCITY_MAX (50.0f)
#define RS05_SPD_MAX (RS05_VELOCITY_MAX)
#define RS05_TORQUE_MAX (5.5f)
#define RS05_KP_MIN (0.0f)
#define RS05_KP_MAX (500.0f)
#define RS05_KD_MIN (0.0f)
#define RS05_KD_MAX (5.0f)
#define RS05_TRAJECTORY_PERIOD_MS (2U)
#define RS05_TRAJECTORY_EPSILON_RAD (0.000001f)

/* 功能：初始化RS05_Init对应的外设、参数和运行状态。 */
HAL_StatusTypeDef RS05_Init(rs05_t *motor                     ,
                            FDCAN_HandleTypeDef *hcan                    , uint8_t id                 ,
                            int direction                       );

/* 功能：构建RS05_BuildSpecial需要发送的通信帧或控制指令。 */
bool RS05_BuildSpecial(rs05_t *motor                       , uint8_t command             );

/* 功能：设置RS05_SetTargetBasic对应的目标值或工作模式。 */
void RS05_SetTargetBasic(rs05_t *motor                      , rs_mode_t mode              , /* 当前工作模式或状态。 */
                         float target                      );

/* 功能：发送RS05_Send对应的数据或控制指令。 */
bool RS05_Send(rs05_t *motor                      );

/* 功能：接收并解析RS05_OnFrame对应的输入数据，更新模块状态。 */
bool RS05_OnFrame(rs05_t *motor                      , const can_frame_t *frame                     );

/* 功能：停止RS05_Stop对应的输出并清理运行状态。 */
bool RS05_Stop(rs05_t *motor                    );

/* 功能：执行RS05_Service对应的周期任务并维护运行状态。 */
void RS05_Service(rs05_t *motor                             );

/* 功能：读取RS05_GetPositionDegrees对应的数据或运行状态。 */
void RS05_GetPositionDegrees(const rs05_t *motor                      ,
                             volatile float *position_degrees                      );

/* 功能：设置RS05_SetTargetMIT对应的 MIT 控制目标。 */
void RS05_SetTargetMIT(rs05_t *motor, rs_mode_t mode, float tau_nm, float output_degrees);
#define RS05_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define RS05_SetTarget(...) RS05_TARGET_SELECT(__VA_ARGS__, RS05_SetTargetMIT, RS05_SetTargetBasic)(__VA_ARGS__)

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
