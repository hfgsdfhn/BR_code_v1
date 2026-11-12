





#ifndef RS01_H
#define RS01_H

#include "bsp_can.h"
#include "pid.h"
#include "trapezoid_planner.h"
#include <stdbool.h>
#include <stdint.h>

#define RS01_SET_ZERO 6U
#define RS01_ZERO_POSITION_TOLERANCE_RAD 0.01f

#define RS01_ENABLE 3U
#define RS01_FALSE 4U
#define RS01_CMD_MOTION 1U
#define RS01_CMD_FEEDBACK 2U


#define RS01_CMD_WRITE_PARAM 0x12U


#define RS01_RUN_MODE_MIT 0U
#define RS01_RUN_MODE_SPEED 2U


#define RS01_IDX_RUN_MODE 0x7005U
#define RS01_IDX_SPD_REF 0x700AU
#define RS01_IDX_LIMIT_CUR 0x7018U
#define RS01_IDX_ACC_RAD 0x7022U
#define RS01_HOST_ID 0xFDU
#ifndef RS_MODE_T_DEFINED
#define RS_MODE_T_DEFINED

typedef enum
{
    RS_MODE_STOP = 0U,
    RS_MODE_DAMPING,
    RS_MODE_SPEED,
    RS_MODE_POSITION,
    RS_MODE_MIT
} rs_mode_t;
#endif


#define RS01_SPEED_PLANNER_RESEED_SEC 0.1f


#define RS01_SPEED_PLANNER_MAX_DT_SEC 0.02f
#define RS01_ANGLE_PERIOD 6.28318530718f
#define RS01_DEG_TO_RAD 0.017453292519943295f

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid;
    uint32_t cascade_tick;
    bool cascade_configured;

    FDCAN_HandleTypeDef *hcan;
    uint8_t id;
    int direction;
    rs_mode_t mode;
    float target;
    float hold_position;
    float reference_position;
    float reference_velocity;
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
    volatile bool enabled; /* Confirmed running and fault-free feedback only. */
    volatile uint8_t feedback_state; /* 0 reset, 1 calibration, 2 running. */
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
    uint8_t run_mode;
} rs01_t;

extern volatile float rs01_trajectory_max_speed_rad_s;
extern volatile float rs01_trajectory_max_acceleration_rad_s2;
extern volatile float rs01_trajectory_time_s;
extern volatile float rs01_speed_max_acceleration_rad_s2;
extern volatile float rs01_speed_limit_current_a;
extern volatile float rs01_speed_accel_rad_s2;
#define RS01_POSITION_MIN (-12.57f)
#define RS01_POSITION_MAX (12.57f)
#define RS01_VELOCITY_MAX (44.0f)
#define RS01_SPD_MAX (RS01_VELOCITY_MAX)
#define RS01_TORQUE_MAX (17.0f)
#define RS01_TRAJECTORY_PERIOD_MS (2U)
#define RS01_TRAJECTORY_EPSILON_RAD (0.000001f)


HAL_StatusTypeDef RS01_Init(rs01_t *motor                     ,
                            FDCAN_HandleTypeDef *hcan                    , uint8_t id                 ,
                            int direction                       );

bool RS01_BuildSpecial(rs01_t *motor                       , uint8_t command             );

void RS01_SetTargetBasic(rs01_t *motor                      , rs_mode_t mode              ,
                         float target                      );

bool RS01_Send(rs01_t *motor                      );

bool RS01_OnFrame(rs01_t *motor                      , const can_frame_t *frame                     );

bool RS01_Stop(rs01_t *motor                    );

void RS01_Service(rs01_t *motor                             );

void RS01_GetPositionDegrees(const rs01_t *motor                      ,
                             volatile float *position_degrees                      );


void RS01_SetTargetMIT(rs01_t *motor, rs_mode_t mode, float tau_nm, float output_degrees);
#define RS01_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define RS01_SetTarget(...) RS01_TARGET_SELECT(__VA_ARGS__, RS01_SetTargetMIT, RS01_SetTargetBasic)(__VA_ARGS__)


#define RS01_POSITION_PID_KP 4.0f
#define RS01_POSITION_PID_KI 0.0f
#define RS01_POSITION_PID_KD 0.0f
#define RS01_POSITION_PID_I_LIMIT 0.0f
#define RS01_POSITION_SPEED_LIMIT 0.5f
#define RS01_SPEED_PID_KP 1.0f
#define RS01_SPEED_PID_KI 0.1f
#define RS01_SPEED_PID_KD 0.0f
#define RS01_SPEED_PID_I_LIMIT 1.0f
#define RS01_CASCADE_PERIOD_S 0.002f
#define RS01_MIT_POSITION_KP 20.0f
#define RS01_MIT_VELOCITY_KD 1.0f

#endif
