




#ifndef BR_ARM_CONFIG_H
#define BR_ARM_CONFIG_H

#define BR_LEFT_CAN (&hfdcan2)
#define BR_RIGHT_CAN (&hfdcan3)
#define BR_PERIOD_MS 10U
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
#define BR_SHOULDER_KP 30.0f
#define BR_SHOULDER_KD 1.0f
#define BR_ELBOW_KP 25.0f
#define BR_ELBOW_KD 0.8f
#define BR_WRIST_KP 15.0f
#define BR_WRIST_KD 0.5f
#define BR_ROTATION_KP 10.0f
#define BR_ROTATION_KD 0.4f
#define BR_ROOT_RS01_KP 20.0f
#define BR_ROOT_RS01_KD 0.8f
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
#endif
