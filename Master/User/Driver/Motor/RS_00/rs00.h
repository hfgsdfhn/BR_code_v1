/*
 * @file rs00.h
 * @brief rs00模块的用户接口与实现。
 */

#ifndef RS00_H
#define RS00_H

#include "bsp_can.h"
#include "pid.h"
#include "trapezoid_planner.h"
#include <stdbool.h>
#include <stdint.h>

#define RS00_SET_ZERO 6U
#define RS00_ZERO_POSITION_TOLERANCE_RAD 0.01f

#define RS00_ENABLE 3U
#define RS00_FALSE 4U
#define RS00_CMD_MOTION 1U
#define RS00_CMD_FEEDBACK 2U


#define RS00_CMD_WRITE_PARAM 0x12U


#define RS00_RUN_MODE_MIT 0U
#define RS00_RUN_MODE_SPEED 2U


#define RS00_IDX_RUN_MODE 0x7005U
#define RS00_IDX_SPD_REF 0x700AU
#define RS00_IDX_LIMIT_CUR 0x7018U
#define RS00_IDX_ACC_RAD 0x7022U
#define RS00_HOST_ID 0xFDU
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


#define RS00_SPEED_PLANNER_RESEED_SEC 0.1f


#define RS00_SPEED_PLANNER_MAX_DT_SEC 0.02f
#define RS00_ANGLE_PERIOD 6.28318530718f
#define RS00_DEG_TO_RAD 0.017453292519943295f

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid; /* 成员变量。 */
    uint32_t cascade_tick; /* 成员变量。 */
    bool cascade_configured; /* 成员变量。 */
    float mit_kp, mit_kd; /* 代码说明：相关模块实现。 */

    FDCAN_HandleTypeDef *hcan; /* 对应外设句柄。 */
    uint8_t id; /* 成员变量。 */
    int direction; /* 成员变量。 */
    rs_mode_t mode; /* 当前工作模式或状态。 */
    float target; /* 成员变量。 */
    float hold_position; /* 成员变量。 */
    float reference_position; /* 成员变量。 */
    float reference_velocity; /* 成员变量。 */
    float trajectory_time_s; /* 成员变量。 */
    float trajectory_start_position; /* 成员变量。 */
    float trajectory_delta_position; /* 成员变量。 */
    float trajectory_k0; /* 成员变量。 */
    uint32_t trajectory_start_tick; /* 成员变量。 */
    uint32_t trajectory_duration_ms; /* 成员变量。 */
    volatile uint32_t last_feedback_tick; /* 成员变量。 */
    volatile float position; /* 成员变量。 */
    volatile float single_position; /* 成员变量。 */
    volatile float velocity; /* 成员变量。 */
    volatile float torque; /* 成员变量。 */
    volatile uint8_t fault; /* 成员变量。 */
    volatile bool feedback_valid; /* 状态标志。 */
    volatile bool feedback_recovery_pending; /* 成员变量。 */
    volatile bool enabled; /* 代码说明：相关模块实现。 */
    volatile uint8_t feedback_state; /* 代码说明：相关模块实现。 */
    float continuous_position; /* 成员变量。 */
    bool continuous_valid; /* 状态标志。 */
    bool angle_store_pending; /* 成员变量。 */
    volatile bool offline_latched; /* 成员变量。 */
    volatile bool ever_feedback_received; /* 成员变量。 */
    volatile uint32_t reconnect_retry_tick; /* 成员变量。 */
    volatile uint32_t last_poll_tick; /* 成员变量。 */
    volatile uint32_t last_control_tick; /* 成员变量。 */
    bool trajectory_pending; /* 成员变量。 */
    bool trajectory_active; /* 状态标志。 */
    volatile bool startup_hold_active; /* 状态标志。 */
    volatile bool startup_hold_initialized; /* 成员变量。 */
    volatile bool recovery_trajectory; /* 成员变量。 */
    volatile bool recovery_sync_candidate; /* 成员变量。 */
    volatile uint8_t set_zero_position_request; /* 成员变量。 */
    trapezoid_t speed_planner; /* 成员变量。 */
    uint32_t speed_planner_tick; /* 成员变量。 */
    bool speed_planner_valid; /* 状态标志。 */
    float tau_ff_nm; /* 成员变量。 */
    uint8_t run_mode; /* 当前工作模式或状态。 */
} rs00_t;

extern volatile float rs00_trajectory_max_speed_rad_s; /* 成员变量。 */
extern volatile float rs00_trajectory_max_acceleration_rad_s2; /* 成员变量。 */
extern volatile float rs00_trajectory_time_s; /* 成员变量。 */
extern volatile float rs00_speed_max_acceleration_rad_s2; /* 成员变量。 */
extern volatile float rs00_speed_limit_current_a; /* 成员变量。 */
extern volatile float rs00_speed_accel_rad_s2; /* 成员变量。 */
#define RS00_POSITION_MIN (-12.57f)
#define RS00_POSITION_MAX (12.57f)
#define RS00_VELOCITY_MAX (33.0f)
#define RS00_SPD_MAX (RS00_VELOCITY_MAX)
#define RS00_TORQUE_MAX (14.0f)
#define RS00_TRAJECTORY_PERIOD_MS (2U)
#define RS00_TRAJECTORY_EPSILON_RAD (0.000001f)


/* 功能：执行 RS00_Init 函数对应的模块操作。 */
HAL_StatusTypeDef RS00_Init(rs00_t *motor                     , /* 成员变量。 */
                            FDCAN_HandleTypeDef *hcan                    , uint8_t id                 , /* 成员变量。 */
                            int direction                       );

/* 功能：执行 RS00_BuildSpecial 函数对应的模块操作。 */
bool RS00_BuildSpecial(rs00_t *motor                       , uint8_t command             );

/* 功能：执行 RS00_SetTargetBasic 函数对应的模块操作。 */
void RS00_SetTargetBasic(rs00_t *motor                      , rs_mode_t mode              , /* 当前工作模式或状态。 */
                         float target                      );

/* 功能：执行 RS00_Send 函数对应的模块操作。 */
bool RS00_Send(rs00_t *motor                      );

/* 功能：执行 RS00_OnFrame 函数对应的模块操作。 */
bool RS00_OnFrame(rs00_t *motor                      , const can_frame_t *frame                     );

/* 功能：执行 RS00_Stop 函数对应的模块操作。 */
bool RS00_Stop(rs00_t *motor                    );

/* 功能：执行 RS00_Service 函数对应的模块操作。 */
void RS00_Service(rs00_t *motor                             );

/* 功能：执行 RS00_GetPositionDegrees 函数对应的模块操作。 */
void RS00_GetPositionDegrees(const rs00_t *motor                      , /* 成员变量。 */
                             volatile float *position_degrees                      );


/* 功能：执行 RS00_SetTargetMIT 函数对应的模块操作。 */
void RS00_SetTargetMIT(rs00_t *motor, rs_mode_t mode, float tau_nm, float output_degrees);
#define RS00_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define RS00_SetTarget(...) RS00_TARGET_SELECT(__VA_ARGS__, RS00_SetTargetMIT, RS00_SetTargetBasic)(__VA_ARGS__)


#define RS00_POSITION_PID_KP 4.0f
#define RS00_POSITION_PID_KI 0.0f
#define RS00_POSITION_PID_KD 0.0f
#define RS00_POSITION_PID_I_LIMIT 0.0f
#define RS00_POSITION_SPEED_LIMIT 0.5f
#define RS00_SPEED_PID_KP 1.0f
#define RS00_SPEED_PID_KI 0.1f
#define RS00_SPEED_PID_KD 0.0f
#define RS00_SPEED_PID_I_LIMIT 1.0f
#define RS00_CASCADE_PERIOD_S 0.002f
#define RS00_MIT_POSITION_KP 20.0f
#define RS00_MIT_VELOCITY_KD 1.0f

#endif
