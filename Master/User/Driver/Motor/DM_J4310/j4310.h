/*
 * @file j4310.h
 * @brief j4310模块的用户接口与实现。
 */

#ifndef J4310_H
#define J4310_H

#include "bsp_can.h"
#include "pid.h"
#include "trapezoid_planner.h"
#include <stdbool.h>
#include <stdint.h>


#define J4310_SPEED_PLANNER_RESEED_SEC 0.1f
#define J4310_SPEED_PLANNER_MAX_DT_SEC 0.02f
#define J4310_ENABLE_COMMAND 0xFCU
#define J4310_DISABLE_COMMAND 0xFDU
#define J4310_DEG_TO_RAD 0.017453292519943295f
#define J4310_SET_ZERO 0xFEU
#define J4310_ZERO_POSITION_TOLERANCE_RAD 0.01f
#define J4310_PI 3.14159265359f
#define J4310_TWO_PI 6.28318530718f
#define J4310_POSITION_MAX 12.5f
#define J4310_SPEED_MAX 30.0f
#define J4310_TORQUE_MAX 10.0f
#define J4310_KP_MAX 500.0f
#define J4310_KD_MAX 5.0f
#define J4310_STATE_ENABLED 0x01U
#define J4310_STARTUP_HOLD_KP 30.0f
#define J4310_STARTUP_HOLD_KD 0.95f
#define J4310_STARTUP_ENABLE_RETRY_CYCLES 20U
#define J4310_SPD_MAX_RAD_S J4310_SPEED_MAX
#define J4310_SPEED_ID_OFFSET 0x200U
#define J4310_TRAJECTORY_PERIOD_MS 1U
#define J4310_TRAJECTORY_EPSILON_RAD 0.0001f
#define J4310_DEFAULT_MASTER_ID 0x04U
#define J4310_FEEDBACK_TIMEOUT_CYCLES 20U

typedef struct
{
    bool enabled; /* 状态标志。 */
    bool started; /* 成员变量。 */
    float base_kp; /* 成员变量。 */
    float base_kd; /* 成员变量。 */
    float applied_kp; /* 成员变量。 */
    float applied_kd; /* 成员变量。 */
    float previous_error; /* 成员变量。 */
    float previous_abs_error; /* 成员变量。 */
    float convergence_rate; /* 成员变量。 */
} j4310_online_mit_state_t;

typedef enum
{
    J4310_MODE_STOP = 0U, /* 当前工作模式或状态。 */
    J4310_MODE_DAMPING, /* 当前工作模式或状态。 */
    J4310_MODE_SPEED, /* 当前工作模式或状态。 */
    J4310_MODE_POSITION, /* 当前工作模式或状态。 */
    J4310_MODE_MIT /* 当前工作模式或状态。 */
} j4310_mode_t;

typedef struct
{
    uint16_t age_cycles; /* 成员变量。 */
    float position_rad; /* 成员变量。 */
    float speed_rad_s; /* 成员变量。 */
    float torque_nm; /* 成员变量。 */
    uint8_t state; /* 当前工作模式或状态。 */
    uint8_t mos_temperature; /* 成员变量。 */
    uint8_t rotor_temperature; /* 成员变量。 */
    bool received; /* 成员变量。 */
} j4310_feedback_t;

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid; /* 成员变量。 */
    uint32_t cascade_tick; /* 成员变量。 */
    bool cascade_configured; /* 成员变量。 */
    float mit_kp, mit_kd; /* 代码说明：相关模块实现。 */

    FDCAN_HandleTypeDef *hcan; /* 对应外设句柄。 */
    uint8_t motor_id; /* 成员变量。 */
    uint16_t master_id; /* 成员变量。 */
    int direction; /* 成员变量。 */
    j4310_mode_t mode; /* 当前工作模式或状态。 */
    float target_position_rad; /* 成员变量。 */
    float target_speed_rad_s; /* 成员变量。 */
    float reference_position_rad; /* 成员变量。 */
    float reference_speed_rad_s; /* 成员变量。 */
    float trajectory_time_s; /* 成员变量。 */
    float trajectory_start_rad; /* 成员变量。 */
    float trajectory_k0; /* 成员变量。 */
    uint32_t trajectory_started_at; /* 成员变量。 */
    uint32_t trajectory_duration_ms; /* 成员变量。 */
    float target_torque_nm; /* 成员变量。 */
    float kp; /* 成员变量。 */
    float kd; /* 成员变量。 */
    j4310_feedback_t feedback; /* 成员变量。 */
    float last_raw_position_rad; /* 成员变量。 */
    float position_offset_rad; /* 成员变量。 */
    uint8_t startup_enable_countdown; /* 数据长度或数量。 */
    bool position_tracking_valid; /* 状态标志。 */
    bool offline_latched; /* 成员变量。 */
    bool fault_active; /* 状态标志。 */
    bool fault_disable_sent; /* 成员变量。 */
    bool fault_recovery_pending; /* 成员变量。 */
    bool trajectory_pending; /* 成员变量。 */
    bool trajectory_active; /* 状态标志。 */
    trapezoid_t speed_planner; /* 成员变量。 */
    uint32_t speed_planner_tick; /* 成员变量。 */
    bool speed_planner_valid; /* 状态标志。 */
    bool startup_hold_pending; /* 成员变量。 */
    bool startup_hold_active; /* 状态标志。 */
    bool recovery_trajectory; /* 成员变量。 */
    volatile uint8_t set_zero_position_request; /* 成员变量。 */
} j4310_t;

extern volatile float j4310_trajectory_max_speed_rad_s; /* 成员变量。 */
extern volatile float j4310_trajectory_max_acceleration_rad_s2; /* 成员变量。 */
extern volatile float j4310_trajectory_time_s; /* 成员变量。 */

extern volatile float j4310_speed_max_acceleration_rad_s2; /* 成员变量。 */

extern volatile float j4310_tau_ff_nm; /* 成员变量。 */


/* 功能：执行 J4310_Init 函数对应的模块操作。 */
HAL_StatusTypeDef J4310_Init(j4310_t *motor, FDCAN_HandleTypeDef *hcan, /* 对应外设句柄。 */
                             uint8_t motor_id, int direction);


/* 功能：执行 J4310_SetMode 函数对应的模块操作。 */
void J4310_SetMode(j4310_t *motor                         , j4310_mode_t mode                    );


/* 功能：执行 J4310_SetTargetMode 函数对应的模块操作。 */
void J4310_SetTargetMode(j4310_t *motor                       , j4310_mode_t mode              , /* 当前工作模式或状态。 */
                         float target                             , /* 成员变量。 */
                         float speed_rad_s                    , float kp                , /* 成员变量。 */
                         float kd                , float torque_nm                  );


/* 功能：执行 J4310_Stop 函数对应的模块操作。 */
bool J4310_Stop(j4310_t *motor                       );


/* 功能：执行 J4310_RequestEnable 函数对应的模块操作。 */
void J4310_RequestEnable(j4310_t *motor                            );


/* 功能：执行 J4310_SetTargetBasic 函数对应的模块操作。 */
void J4310_SetTargetBasic(j4310_t *motor, j4310_mode_t mode, float target);

/* 功能：执行 J4310_SetMitTarget 函数对应的模块操作。 */
void J4310_SetMitTarget(j4310_t *motor                              , /* 成员变量。 */
                        float position_degrees                          , /* 成员变量。 */
                        float speed_rad_s                             , /* 成员变量。 */
                        float kp                            , /* 成员变量。 */
                        float kd                            , /* 成员变量。 */
                        float torque_nm                             );


/* 功能：执行 J4310_SetPositionDegrees 函数对应的模块操作。 */
void J4310_SetPositionDegrees(j4310_t *motor                         , /* 成员变量。 */
                              float degrees                         , /* 成员变量。 */
                              float kp                           , /* 成员变量。 */
                              float kd                           , /* 成员变量。 */
                              float torque_nm                             );


/* 功能：执行 J4310_OnFrame 函数对应的模块操作。 */
bool J4310_OnFrame(j4310_t *motor                         , /* 成员变量。 */
                   const can_frame_t *frame                      );

/* 功能：执行 J4310_GetPositionDegrees 函数对应的模块操作。 */
void J4310_GetPositionDegrees(const j4310_t *motor                         , /* 成员变量。 */
                              volatile float *position_degrees                     );


/* 功能：执行 J4310_BuildEnable 函数对应的模块操作。 */
void J4310_BuildEnable(const j4310_t *motor                              , /* 成员变量。 */
                       can_frame_t *frame                              );


/* 功能：执行 J4310_BuildDisable 函数对应的模块操作。 */
void J4310_BuildDisable(const j4310_t *motor                              , /* 成员变量。 */
                        can_frame_t *frame                              );


/* 功能：执行 J4310_BuildMit 函数对应的模块操作。 */
void J4310_BuildMit(const j4310_t *motor                             , /* 成员变量。 */
                    float position_rad                               , /* 成员变量。 */
                    float speed_rad_s                                 , /* 成员变量。 */
                    float kp                           , float kd                           , /* 成员变量。 */
                    float torque_nm                             , /* 成员变量。 */
                    can_frame_t *frame                                );


/* 功能：执行 J4310_Service 函数对应的模块操作。 */
void J4310_Service(j4310_t *motor, float additional_torque_nm, uint32_t now);
/* 功能：执行 J4310_BuildControlFrame 函数对应的模块操作。 */
bool J4310_BuildControlFrame(j4310_t *motor                             , /* 成员变量。 */
                             float additional_torque_nm                       , /* 成员变量。 */
                             uint32_t now                         , /* 成员变量。 */
                             can_frame_t *frame                               );


/* 功能：执行 J4310_SetTargetMIT 函数对应的模块操作。 */
void J4310_SetTargetMIT(j4310_t *motor, j4310_mode_t mode, float tau_nm, float output_degrees);
#define J4310_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define J4310_SetTarget(...) J4310_TARGET_SELECT(__VA_ARGS__, J4310_SetTargetMIT, J4310_SetTargetBasic)(__VA_ARGS__)


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
