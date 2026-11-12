





#ifndef M2006_H
#define M2006_H

#include "bsp_can.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#define M2006_TARGET_UPDATE_PERIOD_S 0.002f
#define M2006_DEG_TO_RAD 0.017453292519943295f
#define M2006_ENABLE 1U
#define M2006_DISABLE 0U
#define M2006_MIN_TRAJECTORY_S 0.01f
#define M2006_TRAJECTORY_EPSILON 0.000001f
#define M2006_DAMPING_GAIN_A_PER_RAD_S 0.5f
#define M2006_DAMPING_CURRENT_LIMIT_A 2.0f
#define M2006_FEEDBACK_TIMEOUT_CYCLES 250U
#define M2006_MOTOR_COUNT 8U
#define M2006_ENCODER_COUNT 8192.0f
#define M2006_REDUCTION_RATIO 36.0f
#define M2006_CURRENT_SCALE (10.0f / 10000.0f)
#define M2006_CURRENT_RAW_LIMIT 10000.0f
#define M2006_CUR_MAX_A 10.0f
#define M2006_SPD_MAX_RAD_S 5.235987756f
#define M2006_ZERO_STABLE_FRAMES 1U
#define M2006_ZERO_MAX_STEP_COUNTS 128
#define M2006_ZERO_MAX_SPEED_RPM 30
#define RPM_TO_RAD_S 0.10471975512f
#define TWO_PI 6.28318530718f

typedef enum
{
    M2006_MODE_STOP = 0U,
    M2006_MODE_DAMPING,
    M2006_MODE_SPEED,
    M2006_MODE_POSITION,
    M2006_MODE_CURRENT
} m2006_mode_t;

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target,
                        float position_deg);

    FDCAN_HandleTypeDef *hcan;
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
    float target;
    float target_current_a;
    float target_position_rad;
    float target_speed_rad_s;
    float reference_position_rad;
    float reference_speed_rad_s;
    float reference_acceleration_rad_s2;
    float trajectory_start_rad;
    float trajectory_delta_rad;
    float trajectory_k0;
    float trajectory_duration_s;
    float trajectory_elapsed_s;
    pid_t position_pid;
    pid_t speed_pid;
    m2006_mode_t mode;
    uint8_t zero_stable_frames;
    bool received;
    bool feedback_online;
    bool speed_reference_valid;
    bool trajectory_pending;
    bool trajectory_active;
    bool startup_hold_active;
} m2006_t;

extern volatile float m2006_trajectory_max_speed_rad_s;
extern volatile float m2006_trajectory_max_acceleration_rad_s2;
extern volatile float m2006_trajectory_time_s;


HAL_StatusTypeDef M2006_Init(m2006_t *motor                        ,
                             FDCAN_HandleTypeDef *hcan                  ,
                             uint8_t id                                    ,
                             int direction                                 );


bool M2006_OnFrame(m2006_t *motor                         ,
                   const can_frame_t *frame                      );


void M2006_SetTarget(m2006_t *motor                         ,
                     m2006_mode_t mode                          ,
                     float target                               );


bool M2006_Stop(m2006_t *motor                       );


void M2006_SetPositionDegrees(m2006_t *motor                     ,
                              float degrees                          );


int16_t M2006_Calc(m2006_t *motor                             ,
                   float dt_s                          );


void M2006_BuildGroupFrame(uint8_t first_id                               ,
                           const int16_t current[4]                             ,
                           can_frame_t *frame                              );


void M2006_ResetFeedback(m2006_t *motor                            );

void M2006_Service(m2006_t *motor                             ,
                   float dt_s                   );

void M2006_GetPositionDegrees(const m2006_t *motor                         ,
                              volatile float *position_degrees                      );

#ifndef M2006_BUILD_SPECIAL_DECLARED
#define M2006_BUILD_SPECIAL_DECLARED


void M2006_BuildSpecial(m2006_t *motor                        , uint8_t command             );
#endif


void M2006_SetTargetDeferred(m2006_t *motor, m2006_mode_t mode, float target);


#define M2006_POSITION_KP 94.247779608f
#define M2006_POSITION_KI 52.359877560f
#define M2006_POSITION_KD 0.523598776f
#define M2006_POSITION_OUTPUT_LIMIT 5.235987756f
#define M2006_POSITION_INTEGRAL_LIMIT 0.002f
#define M2006_SPEED_KP 3.342253805f
#define M2006_SPEED_KI 2.387324146f
#define M2006_SPEED_KD 0.0f
#define M2006_SPEED_OUTPUT_LIMIT 10.0f
#define M2006_SPEED_INTEGRAL_LIMIT 0.052359878f
#define M2006_POSITION_I_FULL 0.01f
#define M2006_POSITION_I_SEPARATION 0.20f
#define M2006_SPEED_I_FULL 0.209440f
#define M2006_SPEED_I_SEPARATION 3.141593f

#endif
