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

#define ARM_OVERTEMP_PROTECTION_ENABLED 0U
#define ARM_JOINT_COUNT 5U
#define ARM_OVERTEMP_TRIP_TEMPERATURE_C 70.0f
#define ARM_OVERTEMP_CLEAR_TEMPERATURE_C 65.0f
#define ARM_LARM_INDEX 0U
#define ARM_RARM_INDEX 1U
#define larm ARM_LARM_INDEX
#define rarm ARM_RARM_INDEX
#define dcup 0U
#define ucup 1U
#define ARM1_BOARD_INDEX ARM_LARM_INDEX
#define ARM2_BOARD_INDEX ARM_RARM_INDEX
#define ARM_DEG_TO_RAD 0.017453292519943295f
#define ARM_RAD_TO_DEG 57.29577951308232f

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

extern volatile float arm_feedback_angle[2][ARM_JOINT_COUNT];
extern volatile float arm_end_position_world[2][3];
extern volatile float arm_end_target_world[2][3];
extern volatile uint8_t arm_ik_enable[2];
extern volatile uint8_t arm_ik_second_rs00_prefer_small[2];
extern upper_t upper_state;
extern Arm_Pos Arm_larm, Arm_rarm;
extern m2006_t m2006_larm;
extern rs00_t rs00_larm[2], rs00_rarm[2];
extern rs05_t rs05_larm, rs05_rarm;
extern j4310_t j4310_larm, j4310_rarm;
extern rs01_t rs01_rarm;

extern volatile uint8_t larm_manual_tau_enable;
extern volatile float larm_shoulder_tau_nm;
extern volatile float larm_elbow_tau_nm;
extern volatile float larm_wrist_tau_nm;
extern volatile float larm_rotation_tau_nm;
extern volatile uint8_t rarm_manual_tau_enable;
extern volatile float rarm_shoulder_tau_nm;
extern volatile float rarm_elbow_tau_nm;
extern volatile float rarm_wrist_tau_nm;
extern volatile float rarm_rotation_tau_nm;
extern volatile float larm_mass_003_kg;
extern volatile float larm_mass_004_kg;
extern volatile float larm_mass_005_kg;
extern volatile float larm_mass_006_kg;
extern volatile float rarm_mass_003_kg;
extern volatile float rarm_mass_004_kg;
extern volatile float rarm_mass_005_kg;
extern volatile float rarm_mass_006_kg;

void Upper_BindMotors(upper_t *upper);
void Upper_OnFrame(upper_t *upper, const can_frame_t *frame);
void Upper_Service(uint32_t now);
bool Upper_IsArmOvertemperature(uint8_t arm);

typedef enum { ARM_JOINT_ROOT, ARM_JOINT_SHOULDER, ARM_JOINT_ELBOW, ARM_JOINT_WRIST, ARM_JOINT_ROTATION } arm_joint_t;
bool Upper_SetJointMIT(uint8_t arm, arm_joint_t joint, float tau_nm, float output_degrees);
bool Upper_SetJointModeTarget(uint8_t arm, arm_joint_t joint, arm_mode_t mode, float target_degrees);
bool Upper_SetJointDegrees(uint8_t arm, arm_joint_t joint, float degrees);
void Upper_QueueArmPositions(bool enable);
void Upper_SetArmIKTarget(uint8_t arm, float x_cm, float y_cm, float z_cm);
void Upper_SetArmIKEnabled(uint8_t arm, bool enable);
void Upper_RefreshArmIK(uint8_t arm);
bool apply_arm(uint8_t arm, uint8_t joint_index, float target_degrees);
bool apply_cup(uint8_t cup, uint8_t index, float target_degrees);

#endif
