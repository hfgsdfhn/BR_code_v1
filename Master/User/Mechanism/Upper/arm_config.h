/*
 * @file arm_config.h
 * @brief arm_config 模块的用户接口与实现。
 */

#ifndef BR_ARM_CONFIG_H
#define BR_ARM_CONFIG_H
#include "../../Driver/Motor/DM_J4310/j4310.h"
#include "../../Driver/Motor/RS_00/rs00.h"
#include "../../Driver/Motor/RS_01/rs01.h"
#include "../../Driver/Motor/RS_05/rs05.h"

#define BR_LEFT_CAN (&hfdcan2)
#define BR_RIGHT_CAN (&hfdcan3)
#define BR_PERIOD_MS 10U

#define BR_ROOT_M2006_TRAJECTORY_TIME_S 0.0f
#define BR_ROOT_RS01_TRAJECTORY_TIME_S 0.0f
#define BR_BIG_ARM_RS00_TRAJECTORY_TIME_S 0.0f
#define BR_SMALL_ARM_RS00_TRAJECTORY_TIME_S 0.0f
#define BR_J4310_TRAJECTORY_TIME_S 0.0f
#define BR_RS05_TRAJECTORY_TIME_S 0.0f

#define BR_FEEDBACK_TIMEOUT_MS 100U
#define BR_COMMAND_TIMEOUT_MS 100U
#define BR_ENABLE_RETRY_MS 20U
#define BR_RS_HOST_ID 0xFDU
#define BR_SHOULDER_ID 2U
#define BR_ELBOW_ID 3U
#define BR_WRIST_ID 4U
#define BR_ROTATION_ID 5U
#define BR_DM_FEEDBACK_ID 0U
#define BR_ROOT_RS01_ID 1U
#define BR_LEFT_DIRECTION {1.0f, 1.0f, 1.0f, 1.0f}
#define BR_RIGHT_DIRECTION {1.0f, 1.0f, 1.0f, 1.0f}

#define BR_LEFT_ZERO_RAD {0.0f, 0.0f, 0.0f, 0.0f}
#define BR_RIGHT_ZERO_RAD {0.0f, 0.0f, 0.0f, 0.0f}
#define BR_LEFT_JOINT_PER_MOTOR {1.0f, 1.0f, 1.0f, 1.0f}
#define BR_RIGHT_JOINT_PER_MOTOR {1.0f, 1.0f, 1.0f, 1.0f}

extern volatile float br_shoulder_kp;
extern volatile float br_shoulder_kd;
extern volatile float br_elbow_kp;
extern volatile float br_elbow_kd;
extern volatile float br_wrist_kp;
extern volatile float br_wrist_kd;
extern volatile float br_rotation_kp;
extern volatile float br_rotation_kd;
extern volatile float br_root_rs01_kp;
extern volatile float br_root_rs01_kd;
extern volatile float br_root_m2006_position_kp;
extern volatile float br_root_m2006_position_ki;
extern volatile float br_root_m2006_position_kd;
extern volatile float br_root_m2006_speed_kp;
extern volatile float br_root_m2006_speed_ki;
extern volatile float br_root_m2006_speed_kd;

#define BR_ROOT_RS01_DIRECTION 1.0f
#define BR_JOINT_MIN_RAD {-3.14159265f, -3.14159265f, -3.14159265f, -3.14159265f}
#define BR_JOINT_MAX_RAD {3.14159265f, 3.14159265f, 3.14159265f, 3.14159265f}
#define BR_FF_LIMIT_NM {10.0f, 10.0f, 5.0f, 3.0f}
#define BR_FF_SLEW_NM_S 20.0f
#define BR_POSITION_SLEW_RAD_S 0.5f
#define BR_RS_POSITION_MAX 12.5f
#define BR_RS_KP_MAX 500.0f
#define BR_RS_KD_MAX 5.0f
#define BR_RS00_SPEED_MAX 33.0f
#define BR_RS00_TORQUE_MAX 14.0f
#define BR_RS05_SPEED_MAX 50.0f
#define BR_RS05_TORQUE_MAX 5.5f
#define BR_RS01_SPEED_MAX 44.0f
#define BR_RS01_TORQUE_MAX 17.0f
#define BR_DM_POSITION_MAX 12.5f
#define BR_DM_SPEED_MAX 30.0f
#define BR_DM_TORQUE_MAX 10.0f
#define BR_DM_KP_MAX 500.0f
#define BR_DM_KD_MAX 5.0f
#define BR_CAN2_M2006_DIRECTIONS {1, 1, 1, 1}
#define BR_CAN3_M2006_DIRECTIONS {1, 1}
#define BR_CAN2_M3508_DIRECTION 1

#define BR_CAN2_M2006_IDS {1U, 2U, 3U}
#define BR_CAN2_M3508_ID 5U
#define BR_CAN3_M2006_IDS {2U, 3U}

#define BR_LEFT_MODEL_CALIBRATED 0
#define BR_RIGHT_MODEL_CALIBRATED 0
#define BR_LEFT_LINKS                                                                                                  \
    {                                                                                                                  \
        {                                                                                                              \
            .mass_kg = 0.0f                                                                                            \
        }                                                                                                              \
    }
#define BR_RIGHT_LINKS                                                                                                 \
    {                                                                                                                  \
        {                                                                                                              \
            .mass_kg = 0.0f                                                                                            \
        }                                                                                                              \
    }
#define BR_LEFT_GRAVITY_BASE {0.0f, 0.0f, -9.80665f}
#define BR_RIGHT_GRAVITY_BASE {0.0f, 0.0f, -9.80665f}

#define BR_POSITION_PID_KP 4.0f
#define BR_POSITION_PID_KI 0.0f
#define BR_POSITION_PID_KD 0.0f
#define BR_POSITION_PID_I_LIMIT 0.0f
#define BR_SPEED_PID_KP 1.0f
#define BR_SPEED_PID_KI 0.1f
#define BR_SPEED_PID_KD 0.0f
#define BR_SPEED_PID_I_LIMIT 1.0f
#define BR_CASCADE_TORQUE_LIMIT_NM 5.0f

#define BR_MIT_ADD_MODEL_GRAVITY 0
#define BR_PAYLOAD_MASS_KG 0.0f
#define BR_PAYLOAD_COM_M {0.0f, 0.0f, 0.0f}

#define BR_LEFT_ROOT_M2006_INDEX 0U
#define BR_ROOT_MIN_RAD (-3.14159265f)
#define BR_ROOT_MAX_RAD 3.14159265f
#define BR_ROOT_FF_CURRENT_LIMIT_A 1.0f
#define BR_ROOT_FF_SLEW_A_S 2.0f
#define BR_LEFT_ROOT_JOINT_PER_OUTPUT 1.0f

#define BR_LEFT_ROOT_START_RAD 0.0f
#define BR_LEFT_ROOT_POSE_CALIBRATED 0

#define BR_LEFT_ROOT_NM_PER_A 0.0f
#define BR_RIGHT_ROOT_ZERO_RAD 0.0f
#define BR_RIGHT_ROOT_JOINT_PER_MOTOR 1.0f
#define BR_ROOT_FF_LIMIT_NM 10.0f

extern rs00_t rs00_larm[2], rs00_rarm[2];
extern rs05_t rs05_larm, rs05_rarm;
extern j4310_t j4310_larm, j4310_rarm;
extern rs01_t rs01_rarm;

/* 功能：处理configure_motor_gains对应的输入并更新相关状态。 */
static inline void configure_motor_gains(void)
{
    rs00_larm[0].mit_kp = br_shoulder_kp;
    rs00_larm[0].mit_kd = br_shoulder_kd;
    rs00_rarm[0].mit_kp = br_shoulder_kp;
    rs00_rarm[0].mit_kd = br_shoulder_kd;
    rs00_larm[1].mit_kp = br_elbow_kp;
    rs00_larm[1].mit_kd = br_elbow_kd;
    rs00_rarm[1].mit_kp = br_elbow_kp;
    rs00_rarm[1].mit_kd = br_elbow_kd;
    j4310_larm.mit_kp = br_wrist_kp;
    j4310_larm.mit_kd = br_wrist_kd;
    j4310_rarm.mit_kp = br_wrist_kp;
    j4310_rarm.mit_kd = br_wrist_kd;
    rs05_larm.mit_kp = br_rotation_kp;
    rs05_larm.mit_kd = br_rotation_kd;
    rs05_rarm.mit_kp = br_rotation_kp;
    rs05_rarm.mit_kd = br_rotation_kd;
    rs01_rarm.mit_kp = br_root_rs01_kp;
    rs01_rarm.mit_kd = br_root_rs01_kd;

}

#endif
