/*
 * @file m3508.h
 * @brief m3508模块的用户接口与实现。
 */

#ifndef M3508_H
#define M3508_H

#include "bsp_can.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#define M3508_ENABLE 1U
#define M3508_DISABLE 0U
#define M3508_DEG_TO_RAD 0.017453292519943295f
#define M3508_MIN_TRAJECTORY_S 0.01f
#define M3508_TRAJECTORY_EPSILON 0.000001f
#define M3508_FEEDBACK_TIMEOUT_CYCLES 250U
#define M3508_MOTOR_COUNT 8U
#define M3508_ENCODER_COUNT 8192.0f
#define M3508_REDUCTION_RATIO (3591.0f / 187.0f)
#define M3508_CURRENT_SCALE (20.0f / 16384.0f)
#define M3508_CURRENT_RAW_LIMIT 16384.0f
#define M3508_CUR_MAX_A 20.0f
#define M3508_SPD_MAX_RAD_S 31.416f
#define M3508_ZERO_STABLE_FRAMES 1U
#define M3508_ZERO_MAX_STEP_COUNTS 128
#define M3508_ZERO_MAX_SPEED_RPM 30
#define RPM_TO_RAD_S 0.10471975512f
#define TWO_PI 6.28318530718f

typedef enum
{
    M3508_MODE_STOP = 0U, /* 当前工作模式或状态。 */
    M3508_MODE_DAMPING, /* 当前工作模式或状态。 */
    M3508_MODE_SPEED, /* 当前工作模式或状态。 */
    M3508_MODE_POSITION, /* 当前工作模式或状态。 */
    M3508_MODE_CURRENT /* 当前工作模式或状态。 */
} m3508_mode_t;

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, /* 成员变量。 */
                        float position_deg);

    FDCAN_HandleTypeDef *hcan; /* 对应外设句柄。 */
    uint16_t feedback_age_cycles; /* 成员变量。 */
    uint8_t id; /* 成员变量。 */
    int direction; /* 成员变量。 */
    uint16_t encoder; /* 成员变量。 */
    uint16_t last_encoder; /* 成员变量。 */
    int32_t total_encoder; /* 成员变量。 */
    int16_t speed_rpm; /* 成员变量。 */
    int16_t current_raw; /* 成员变量。 */
    float position_rad; /* 成员变量。 */
    float speed_rad_s; /* 成员变量。 */
    float target_current_a; /* 成员变量。 */
    float target_position_rad; /* 成员变量。 */
    float target_speed_rad_s; /* 成员变量。 */
    float reference_position_rad; /* 成员变量。 */
    float reference_speed_rad_s; /* 成员变量。 */
    float reference_acceleration_rad_s2; /* 成员变量。 */
    float trajectory_time_s; /* 成员变量。 */
    float trajectory_start_rad; /* 成员变量。 */
    float trajectory_delta_rad; /* 成员变量。 */
    float trajectory_k0; /* 成员变量。 */
    float trajectory_duration_s; /* 成员变量。 */
    float trajectory_elapsed_s; /* 成员变量。 */
    pid_t position_pid; /* 成员变量。 */
    pid_t speed_pid; /* 成员变量。 */
    pid_t damping_pid; /* 成员变量。 */
    m3508_mode_t mode; /* 当前工作模式或状态。 */
    uint8_t zero_stable_frames; /* 数据缓冲区或数据帧。 */
    bool received; /* 成员变量。 */
    bool feedback_online; /* 状态标志。 */
    bool speed_reference_valid; /* 状态标志。 */
    bool trajectory_pending; /* 成员变量。 */
    bool trajectory_active; /* 状态标志。 */
    bool startup_hold_active; /* 状态标志。 */
} m3508_t;

extern volatile float m3508_trajectory_max_speed_rad_s; /* 成员变量。 */
extern volatile float m3508_trajectory_max_acceleration_rad_s2; /* 成员变量。 */
extern volatile float m3508_trajectory_time_s; /* 成员变量。 */


/* 功能：初始化模块并准备外设资源。 */
HAL_StatusTypeDef M3508_Init(m3508_t *motor                        , /* 成员变量。 */
                             FDCAN_HandleTypeDef *hcan                  , /* 对应外设句柄。 */
                             uint8_t id                                    , /* 成员变量。 */
                             int direction                                  );


/* 功能：接收并解析一帧通信数据。 */
bool M3508_OnFrame(m3508_t *motor                         , /* 成员变量。 */
                   const can_frame_t *frame                      );


/* 功能：设置设备目标值。 */
void M3508_SetTarget(m3508_t *motor                         , /* 成员变量。 */
                     m3508_mode_t mode                          , /* 当前工作模式或状态。 */
                     float target                               );


/* 功能：停止设备输出并清理状态。 */
bool M3508_Stop(m3508_t *motor                       );


/* 功能：执行 M3508_SetPairPositionDegrees 模块操作并维护相关状态。 */
void M3508_SetPairPositionDegrees(m3508_t motors[2]                         , /* 成员变量。 */
                                  float degrees                            );


/* 功能：执行 M3508_Calc 模块操作并维护相关状态。 */
int16_t M3508_Calc(m3508_t *motor                             , /* 成员变量。 */
                   float dt_s                          );


/* 功能：构建待发送的通信帧。 */
void M3508_BuildGroupFrame(uint8_t first_id                               , /* 成员变量。 */
                           const int16_t current[4]                             , /* 成员变量。 */
                           can_frame_t *frame                              );


/* 功能：执行 M3508_ResetFeedback 模块操作并维护相关状态。 */
void M3508_ResetFeedback(m3508_t *motor                            );

/* 功能：执行周期性服务并维护运行状态。 */
void M3508_Service(m3508_t *motor                             , /* 成员变量。 */
                   float dt_s                   );

/* 功能：读取设备当前位置。 */
void M3508_GetPositionDegrees(const m3508_t *motor                         , /* 成员变量。 */
                              volatile float *position_degrees                      );

#ifndef M3508_BUILD_SPECIAL_DECLARED
#define M3508_BUILD_SPECIAL_DECLARED


/* 功能：构建待发送的通信帧。 */
void M3508_BuildSpecial(m3508_t *motor                        , uint8_t command             );
#endif


/* 功能：设置设备目标值。 */
void M3508_SetTargetDeferred(m3508_t *motor, m3508_mode_t mode, float target);


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
