#ifndef MECHANISM_UPPER_H
#define MECHANISM_UPPER_H
#include "../../Algorithm/Gravity/gravity.h"
#include "arm_config.h"
#include "bsp_can.h"
#include "j4310.h"
#include "m2006.h"
#include "m3508.h"
#include "rs00.h"
#include "rs05.h"
#include <stdbool.h>
#define ARM_JOINT_COUNT 5U
#define ARM_LARM_INDEX 0U /* CAN2 左臂 */
#define ARM1_BOARD_INDEX ARM_LARM_INDEX
#define ARM_RARM_INDEX 1U /* CAN3 右臂 */
#define ARM2_BOARD_INDEX ARM_RARM_INDEX
#define ARM_DEG_TO_RAD 0.017453292519943295f
#define ARM_RAD_TO_DEG 57.29577951308232f
/* These five motors belong to an arm. Public positions are joint degrees. */
typedef struct
{
    float RS05, J4310, RS00_2, RS00_1, Root;
} Arm_Pos;
/* Telemetry order retained: rotation, wrist, elbow, shoulder, root. */
typedef struct
{
    volatile uint8_t online;
    volatile uint32_t last_tick;
    volatile int16_t joint_mrad[ARM_JOINT_COUNT], speed_centi_rad_s[ARM_JOINT_COUNT], torque_centi_nm[ARM_JOINT_COUNT],
        temperature_c[ARM_JOINT_COUNT];
    volatile uint16_t fault[ARM_JOINT_COUNT], frame_count;
    volatile bool gravity_ready;
    volatile float root_ff_current_a;               /* Left M2006 only; gravity_tau_nm remains Nm. */
    volatile float gravity_tau_nm[ARM_JOINT_COUNT]; /* same telemetry order, motor-side FF */
} upper_board_t;
typedef struct
{
    upper_board_t board[2];
    volatile uint8_t enable;
} upper_t;
/* CAN2 左臂、CAN3 右臂；RS00 下标 0=大臂，1=小臂。 */
extern rs00_t rs00_larm[2], rs00_rarm[2];
extern rs05_t rs05_larm, rs05_rarm;    /* 末端旋转电机。 */
extern j4310_t j4310_larm, j4310_rarm; /* 腕部电机。 */
typedef rs05_t rs02_t;                 /* 共享状态布局，但协议使用 RS02 专用量程。 */
extern rs02_t rs02_rarm;               /* CAN3 右臂根部。 */
extern m2006_t m2006_larm;             /* CAN2 左臂根部；占四台 M2006 中的配置槽。 */
void RS02_SetTargetBasic(rs02_t *motor, rs_mode_t mode, float target);
void RS02_SetTargetMIT(rs02_t *motor, rs_mode_t mode, float tau_nm, float output_degrees);
#define RS02_TARGET_SELECT(_1, _2, _3, _4, NAME, ...) NAME
#define RS02_SetTarget(...) RS02_TARGET_SELECT(__VA_ARGS__, RS02_SetTargetMIT, RS02_SetTargetBasic)(__VA_ARGS__)
/* 辅助电机：CAN2 根部对应槽不可用，CAN3 仅下标 0、1 有效。 */
extern m2006_t m2006_aux[2][4];
extern m3508_t m3508_aux[1];
extern upper_t upper_state;
extern Arm_Pos Arm_larm, Arm_rarm;
void Upper_Init(upper_t *upper);
void Arm_Init(void);
void Upper_Stop(upper_t *upper);
void Upper_OnFrame(upper_t *upper, const can_frame_t *frame);
bool Upper_TxQueueInit(void);
void Upper_TxTask(void *argument);
typedef enum
{
    ARM_JOINT_ROTATION,
    ARM_JOINT_WRIST,
    ARM_JOINT_ELBOW,
    ARM_JOINT_SHOULDER,
    ARM_JOINT_ROOT
} arm_joint_t;
/* 普通 POSITION 使用关节角度；MIT 使用电机输出轴 tau(Nm) 和角度(deg)。 */
bool Upper_SetJointMIT(uint8_t arm, arm_joint_t joint, float tau_nm, float output_degrees);
bool Upper_SetJointModeTarget(uint8_t arm, arm_joint_t joint, rs_mode_t mode, float target);
bool Upper_SetJointDegrees(uint8_t arm, arm_joint_t joint, float degrees);
bool Upper_SetM2006PositionDegrees(uint8_t bus, uint8_t index, float degrees);
bool Upper_SetM3508PositionDegrees(float degrees);
bool Upper_SetArmDegrees(uint8_t arm, const Arm_Pos *position);
void Upper_SetGripSignal(uint8_t arm, bool detected, uint8_t source);
void Upper_QueueArmPositions(bool enable);
void Upper_SendBoardTargets(upper_t *upper, uint8_t arm, const float target_rad[ARM_JOINT_COUNT]);
void Upper_SendTargets(upper_t *upper, const float target_rad[2][ARM_JOINT_COUNT]);
void Upper_GetJointRad(const upper_t *upper, uint8_t arm, float out[ARM_JOINT_COUNT]);
/* Auxiliary motors: independent commands, never enter the arm gravity model.
 * bus is 2 or 3; index is 0..3 (CAN2), 0..1 (CAN3). Targets follow driver units.
 * All auxiliary commands expire unless refreshed within BR_COMMAND_TIMEOUT_MS. */
bool Upper_SetM2006(uint8_t bus, uint8_t index, m2006_mode_t mode, float target);
bool Upper_SetM3508(m3508_mode_t mode, float target);
bool Upper_SetRS02Degrees(float degrees, bool enable);
void Upper_DirectInit(void);
void Upper_DirectService(uint32_t now);
void Upper_DirectFeedback(upper_t *upper, const can_frame_t *frame);
#endif
