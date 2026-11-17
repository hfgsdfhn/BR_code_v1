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

#define BR_ROOT_M2006_TRAJECTORY_TIME_S 0.0f
#define BR_ROOT_RS01_TRAJECTORY_TIME_S 0.0f
#define BR_BIG_ARM_RS00_TRAJECTORY_TIME_S 0.0f
#define BR_SMALL_ARM_RS00_TRAJECTORY_TIME_S 0.0f
#define BR_J4310_TRAJECTORY_TIME_S 0.0f
#define BR_RS05_TRAJECTORY_TIME_S 0.0f
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
#define BR_LEFT_ROOT_JOINT_PER_OUTPUT 1.0f

#define BR_LEFT_ROOT_START_RAD 0.0f

#define BR_LEFT_ROOT_NM_PER_A 0.0f

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
