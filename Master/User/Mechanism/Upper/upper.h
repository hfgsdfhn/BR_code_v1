#ifndef MECHANISM_UPPER_H
#define MECHANISM_UPPER_H
#include "../../Algorithm/Gravity/gravity.h"
#include "arm_config.h"
#include "../../Driver/Motor/DM_J4310/j4310.h"
#include "../../Driver/Motor/DJ_M2006/m2006.h"
#include "../../Driver/Motor/DJ_M3508/m3508.h"
#include "../../Driver/Motor/RS_00/rs00.h"
#include "../../Driver/Motor/RS_01/rs01.h"
#include "../../Driver/Motor/RS_05/rs05.h"
#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>

#define ARM_JOINT_COUNT 5U
#define ARM_LARM_INDEX 0U
#define ARM_RARM_INDEX 1U
#define ARM1_BOARD_INDEX ARM_LARM_INDEX
#define ARM2_BOARD_INDEX ARM_RARM_INDEX
#define ARM_DEG_TO_RAD 0.017453292519943295f
#define ARM_RAD_TO_DEG 57.29577951308232f

/* Joint command modes, independent of motor driver enum values. */
typedef enum
{
    ARM_MODE_STOP = 0,
    ARM_MODE_DAMPING,
    ARM_MODE_SPEED,
    ARM_MODE_POSITION,
    ARM_MODE_MIT
} arm_mode_t;

extern volatile arm_mode_t arm_mode[2][ARM_JOINT_COUNT];

typedef struct { float RS05, J4310, RS00_2, RS00_1, Root; } Arm_Pos;
typedef struct { rs05_t *rotation; j4310_t *wrist; rs00_t *elbow; rs00_t *shoulder; m2006_t *left_root; rs01_t *right_root; } arm_group_t;
typedef struct { volatile uint8_t online; volatile uint32_t last_tick; volatile int16_t joint_mrad[ARM_JOINT_COUNT], speed_centi_rad_s[ARM_JOINT_COUNT], torque_centi_nm[ARM_JOINT_COUNT], temperature_c[ARM_JOINT_COUNT]; volatile uint16_t fault[ARM_JOINT_COUNT], frame_count; volatile bool gravity_ready; volatile float gravity_tau_nm[ARM_JOINT_COUNT]; } upper_board_t;
typedef struct { arm_group_t arm[2]; upper_board_t board[2]; volatile uint8_t enable; } upper_t;

/* Measured angles in degrees, updated on received feedback. */
extern volatile float arm_feedback_angle[2][ARM_JOINT_COUNT];
extern upper_t upper_state;
extern Arm_Pos Arm_larm, Arm_rarm;
extern m2006_t m2006_larm;
extern rs00_t rs00_larm[2], rs00_rarm[2];
extern rs05_t rs05_larm, rs05_rarm;
extern j4310_t j4310_larm, j4310_rarm;
extern rs01_t rs01_rarm;

/* 鎵嬪姩 MIT 鍓嶉鎵煩锛屽崟浣?N路m銆俁oot 涓嶄娇鐢ㄨ繖浜涘弬鏁般€?*/
extern volatile uint8_t br_manual_tau_enable;
extern volatile float br_shoulder_tau_nm;
extern volatile float br_elbow_tau_nm;
extern volatile float br_wrist_tau_nm;
extern volatile float br_rotation_tau_nm;

void Upper_BindMotors(upper_t *upper);
void Upper_OnFrame(upper_t *upper, const can_frame_t *frame);
void Upper_Service(uint32_t now);

typedef enum { ARM_JOINT_ROOT, ARM_JOINT_SHOULDER, ARM_JOINT_ELBOW, ARM_JOINT_WRIST, ARM_JOINT_ROTATION } arm_joint_t;
bool Upper_SetJointMIT(uint8_t arm, arm_joint_t joint, float tau_nm, float output_degrees);
bool Upper_SetJointModeTarget(uint8_t arm, arm_joint_t joint, arm_mode_t mode, float target_degrees);
bool Upper_SetJointDegrees(uint8_t arm, arm_joint_t joint, float degrees);
void Upper_QueueArmPositions(bool enable);

#endif
