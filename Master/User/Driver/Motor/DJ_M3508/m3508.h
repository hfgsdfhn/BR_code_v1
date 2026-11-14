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
    void (*target_sink)(void *motor, int mode, float target,
                        float position_deg);

    FDCAN_HandleTypeDef *hcan; /* 对应外设句柄。 */
    uint16_t feedback_age_cycles;
    uint8_t id;
    int direction;
    uint16_t encoder;
    uint16_t last_encoder;
    int32_t total_encoder;
    int16_t speed_rpm;
    int16_t current_raw;
    float position_rad;
    float speed_rad_s;
    float target_current_a;
    float target_position_rad;
    float target_speed_rad_s;
    float reference_position_rad;
    float reference_speed_rad_s;
    float reference_acceleration_rad_s2;
    float trajectory_time_s;
    float trajectory_start_rad;
    float trajectory_delta_rad;
    float trajectory_k0;
    float trajectory_duration_s;
    float trajectory_elapsed_s;
    pid_t position_pid;
    pid_t speed_pid;
    pid_t damping_pid;
    m3508_mode_t mode; /* 当前工作模式或状态。 */
    uint8_t zero_stable_frames;
    bool received;
    bool feedback_online;
    bool speed_reference_valid;
    bool trajectory_pending;
    bool trajectory_active;
    bool startup_hold_active;
} m3508_t;

extern volatile float m3508_trajectory_max_speed_rad_s;
extern volatile float m3508_trajectory_max_acceleration_rad_s2;
extern volatile float m3508_trajectory_time_s;

/* 功能：初始化M3508_Init对应的外设、参数和运行状态。 */
HAL_StatusTypeDef M3508_Init(m3508_t *motor                        ,
                             FDCAN_HandleTypeDef *hcan                  , /* 对应外设句柄。 */
                             uint8_t id                                    ,
                             int direction                                  );

/* 功能：接收并解析M3508_OnFrame对应的输入数据，更新模块状态。 */
bool M3508_OnFrame(m3508_t *motor                         ,
                   const can_frame_t *frame                      );

/* 功能：设置M3508_SetTarget对应的目标值或工作模式。 */
void M3508_SetTarget(m3508_t *motor                         ,
                     m3508_mode_t mode                          , /* 当前工作模式或状态。 */
                     float target                               );

/* 功能：停止M3508_Stop对应的输出并清理运行状态。 */
bool M3508_Stop(m3508_t *motor                       );

/* 功能：处理M3508_SetPairPositionDegrees对应的输入并更新相关状态。 */
void M3508_SetPairPositionDegrees(m3508_t motors[2]                         ,
                                  float degrees                            );

/* 功能：计算M3508_Calc对应的控制量或规划结果。 */
int16_t M3508_Calc(m3508_t *motor                             ,
                   float dt_s                          );

/* 功能：构建M3508_BuildGroupFrame需要发送的通信帧或控制指令。 */
void M3508_BuildGroupFrame(uint8_t first_id                               ,
                           const int16_t current[4]                             ,
                           can_frame_t *frame                              );

/* 功能：重置或释放M3508_ResetFeedback对应的内部状态和资源。 */
void M3508_ResetFeedback(m3508_t *motor                            );

/* 功能：执行M3508_Service对应的周期任务并维护运行状态。 */
void M3508_Service(m3508_t *motor                             ,
                   float dt_s                   );

/* 功能：读取M3508_GetPositionDegrees对应的数据或运行状态。 */
void M3508_GetPositionDegrees(const m3508_t *motor                         ,
                              volatile float *position_degrees                      );

#ifndef M3508_BUILD_SPECIAL_DECLARED
#define M3508_BUILD_SPECIAL_DECLARED

/* 功能：构建M3508_BuildSpecial需要发送的通信帧或控制指令。 */
void M3508_BuildSpecial(m3508_t *motor                        , uint8_t command             );
#endif

/* 功能：设置M3508_SetTargetDeferred对应的目标值或工作模式。 */
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
