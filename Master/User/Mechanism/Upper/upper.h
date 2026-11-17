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
extern volatile float arm_end_position_world[2][3]; /* centimetres for debugging */
extern upper_t upper_state;
extern Arm_Pos Arm_larm, Arm_rarm;
extern m2006_t m2006_larm;
extern rs00_t rs00_larm[2], rs00_rarm[2];
extern rs05_t rs05_larm, rs05_rarm;
extern j4310_t j4310_larm, j4310_rarm;
extern rs01_t rs01_rarm;

extern volatile uint8_t br_manual_tau_enable;
extern volatile float br_shoulder_tau_nm;
extern volatile float br_elbow_tau_nm;
extern volatile float br_wrist_tau_nm;
extern volatile float br_rotation_tau_nm;

/* 閸旂喕鍏橀敍姘槱閻炲摙pper_BindMotors鐎电懓绨查惃鍕翻閸忋儱鑻熼弴瀛樻煀閻╃鍙ч悩鑸碘偓浣碘偓?*/
void Upper_BindMotors(upper_t *upper);
/* 閸旂喕鍏橀敍姘复閺€璺鸿嫙鐟欙絾鐎経pper_OnFrame鐎电懓绨查惃鍕翻閸忋儲鏆熼幑顕嗙礉閺囧瓨鏌婂Ο鈥虫健閻樿埖鈧降鈧?*/
void Upper_OnFrame(upper_t *upper, const can_frame_t *frame);
/* 閸旂喕鍏橀敍姘⒔鐞涘pper_Service鐎电懓绨查惃鍕噯閺堢喍鎹㈤崝鈥宠嫙缂佸瓨濮㈡潻鎰攽閻樿埖鈧降鈧?*/
void Upper_Service(uint32_t now);
bool Upper_IsArmOvertemperature(uint8_t arm);

typedef enum { ARM_JOINT_ROOT, ARM_JOINT_SHOULDER, ARM_JOINT_ELBOW, ARM_JOINT_WRIST, ARM_JOINT_ROTATION } arm_joint_t;
/* 閸旂喕鍏橀敍姘愁啎缂冪攷pper_SetJointMIT鐎电懓绨查惃鍕窗閺嶅洤鈧吋鍨ㄥ銉ょ稊濡€崇础閵?*/
bool Upper_SetJointMIT(uint8_t arm, arm_joint_t joint, float tau_nm, float output_degrees);
/* 閸旂喕鍏橀敍姘愁啎缂冪攷pper_SetJointModeTarget鐎电懓绨查惃鍕窗閺嶅洤鈧吋鍨ㄥ銉ょ稊濡€崇础閵?*/
bool Upper_SetJointModeTarget(uint8_t arm, arm_joint_t joint, arm_mode_t mode, float target_degrees);
/* 閸旂喕鍏橀敍姘愁啎缂冪攷pper_SetJointDegrees鐎电懓绨查惃鍕窗閺嶅洤鈧吋鍨ㄥ銉ょ稊濡€崇础閵?*/
bool Upper_SetJointDegrees(uint8_t arm, arm_joint_t joint, float degrees);
/* 閸旂喕鍏橀敍姘殺Upper_QueueArmPositions鐎电懓绨查惃鍕窗閺嶅洦鏆熼幑顔煎閸忋儲澧界悰宀勬Е閸掓ぜ鈧?*/
void Upper_QueueArmPositions(bool enable);
/* 閸旂喕鍏橀敍姘殺閺堢儤顫懛鍌氬彠閼哄倻娲伴弽鍥у晸閸忋儱顕惔鏃傛畱閹笛嗩攽閺堢儤鐎妴?*/
bool apply_arm(uint8_t arm, uint8_t joint_index, float target_degrees);
/* 閸旂喕鍏橀敍姘愁啎缂冾喖鎯涢惄妯兼窗閺嶅洣缍呯純顔艰嫙瀵偓閸氼垰鎯涢惄妯诲付閸掕翰鈧?*/
bool apply_cup(uint8_t cup, uint8_t index, float target_degrees);

#endif
