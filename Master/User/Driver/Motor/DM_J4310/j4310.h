





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
    bool enabled;
    bool started;
    float base_kp;
    float base_kd;
    float applied_kp;
    float applied_kd;
    float previous_error;
    float previous_abs_error;
    float convergence_rate;
} j4310_online_mit_state_t;

typedef enum
{
    J4310_MODE_STOP = 0U,
    J4310_MODE_DAMPING,
    J4310_MODE_SPEED,
    J4310_MODE_POSITION,
    J4310_MODE_MIT
} j4310_mode_t;

typedef struct
{
    uint16_t age_cycles;
    float position_rad;
    float speed_rad_s;
    float torque_nm;
    uint8_t state;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    bool received;
} j4310_feedback_t;

typedef struct
{
    void (*target_sink)(void *motor, int mode, float target, float position_deg);
    pid_t cascade_position_pid, cascade_speed_pid;
    uint32_t cascade_tick;
    bool cascade_configured;

    FDCAN_HandleTypeDef *hcan;
    uint8_t motor_id;
    uint16_t master_id;
    int direction;
    j4310_mode_t mode;
    float target_position_rad;
    float target_speed_rad_s;
    float reference_position_rad;
    float reference_speed_rad_s;
    float trajectory_start_rad;
    float trajectory_k0;
    uint32_t trajectory_started_at;
    uint32_t trajectory_duration_ms;
    float target_torque_nm;
    float kp;
    float kd;
    j4310_feedback_t feedback;
    float last_raw_position_rad;
    float position_offset_rad;
    uint8_t startup_enable_countdown;
    bool position_tracking_valid;
    bool offline_latched;
    bool fault_active;
    bool fault_disable_sent;
    bool fault_recovery_pending;
    bool trajectory_pending;
    bool trajectory_active;
    trapezoid_t speed_planner;
    uint32_t speed_planner_tick;
    bool speed_planner_valid;
    bool startup_hold_pending;
    bool startup_hold_active;
    bool recovery_trajectory;
    volatile uint8_t set_zero_position_request;
} j4310_t;

extern volatile float j4310_trajectory_max_speed_rad_s;
extern volatile float j4310_trajectory_max_acceleration_rad_s2;
extern volatile float j4310_trajectory_time_s;

extern volatile float j4310_speed_max_acceleration_rad_s2;

extern volatile float j4310_tau_ff_nm;


HAL_StatusTypeDef J4310_Init(j4310_t *motor, FDCAN_HandleTypeDef *hcan,
                             uint8_t motor_id, int direction);


void J4310_SetMode(j4310_t *motor                         , j4310_mode_t mode                    );


void J4310_SetTargetMode(j4310_t *motor                       , j4310_mode_t mode              ,
                         float target                             ,
                         float speed_rad_s                    , float kp                ,
                         float kd                , float torque_nm                  );


bool J4310_Stop(j4310_t *motor                       );


void J4310_RequestEnable(j4310_t *motor                            );


void J4310_SetTargetBasic(j4310_t *motor, j4310_mode_t mode, float target);

void J4310_SetMitTarget(j4310_t *motor                              ,
                        float position_degrees                          ,
                        float speed_rad_s                             ,
                        float kp                            ,
                        float kd                            ,
                        float torque_nm                             );


void J4310_SetPositionDegrees(j4310_t *motor                         ,
                              float degrees                         ,
                              float kp                           ,
                              float kd                           ,
                              float torque_nm                             );


bool J4310_OnFrame(j4310_t *motor                         ,
                   const can_frame_t *frame                      );

void J4310_GetPositionDegrees(const j4310_t *motor                         ,
                              volatile float *position_degrees                     );


void J4310_BuildEnable(const j4310_t *motor                              ,
                       can_frame_t *frame                              );


void J4310_BuildDisable(const j4310_t *motor                              ,
                        can_frame_t *frame                              );


void J4310_BuildMit(const j4310_t *motor                             ,
                    float position_rad                               ,
                    float speed_rad_s                                 ,
                    float kp                           , float kd                           ,
                    float torque_nm                             ,
                    can_frame_t *frame                                );


void J4310_Service(j4310_t *motor, float additional_torque_nm, uint32_t now);
bool J4310_BuildControlFrame(j4310_t *motor                             ,
                             float additional_torque_nm                       ,
                             uint32_t now                         ,
                             can_frame_t *frame                               );


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
