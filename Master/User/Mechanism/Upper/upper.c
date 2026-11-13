/*
 * @file upper.c
 * @brief upper模块的用户接口与实现。
 */

#include "upper.h"
#include "cup_config.h"
#include "gravity_model.h"
#include "suction_cup.h"
#include "my_main.h"
#include <math.h>
#include <string.h>

extern volatile float arm_target[2][ARM_JOINT_COUNT];
extern volatile float br_mit_tau[2][ARM_JOINT_COUNT];
extern float br_joint_velocity[2][ARM_JOINT_COUNT];
extern bool br_grip[2];
extern volatile uint8_t arm_joint_zero_calibration[2][ARM_JOINT_COUNT];
static uint8_t zero_calibration_previous[2][ARM_JOINT_COUNT];

/* 模块实现细节。 */
__ALIGNED(32) volatile float arm_feedback_angle[2][ARM_JOINT_COUNT];
upper_t upper_state;
Arm_Pos Arm_larm, Arm_rarm;
static uint32_t last_service_tick;

volatile float br_shoulder_kp = 60.0f;
volatile float br_shoulder_kd = 1.0f;
volatile float br_elbow_kp = 20.0f;
volatile float br_elbow_kd = 1.0f;
volatile float br_wrist_kp = 20.0f;
volatile float br_wrist_kd = 1.0f;
volatile float br_rotation_kp = 20.0f;
volatile float br_rotation_kd = 1.0f;
volatile float br_root_rs01_kp = 20.0f;
volatile float br_root_rs01_kd = 1.0f;
volatile float br_root_m2006_position_kp = 94.247779608f;
volatile float br_root_m2006_position_ki = 52.359877560f;
volatile float br_root_m2006_position_kd = 0.523598776f;
volatile float br_root_m2006_speed_kp = 3.342253805f;
volatile float br_root_m2006_speed_ki = 2.387324146f;
volatile float br_root_m2006_speed_kd = 0.0f;
volatile uint8_t g_rs_recovery_motion_release = 0U;

/* 模块实现细节。 */
volatile uint8_t br_manual_tau_enable = 0U;
volatile float br_shoulder_tau_nm = 0.0f;
volatile float br_elbow_tau_nm = 0.0f;
volatile float br_wrist_tau_nm = 0.0f;
volatile float br_rotation_tau_nm = 0.0f;

static br_gravity_model_t gravity_model[2] = {
    {.link = BR_G_LINKS, .gravity_base = BR_LEFT_GRAVITY_BASE, .payload_mass_kg = BR_PAYLOAD_MASS_KG,
     .payload_com_m = BR_PAYLOAD_COM_M, .calibrated = true},
    {.link = BR_G_LINKS, .gravity_base = BR_RIGHT_GRAVITY_BASE, .payload_mass_kg = BR_PAYLOAD_MASS_KG,
     .payload_com_m = BR_PAYLOAD_COM_M, .calibrated = true}};

/* 模块实现细节。 */
static void Upper_ZeroCalibrationRequest(unsigned arm, unsigned joint)
{
    uint8_t active = arm_joint_zero_calibration[arm][joint] != 0U;
    if (active && !zero_calibration_previous[arm][joint])
    {
        if (joint == ARM_JOINT_ROOT)
        {
            if (arm == ARM_RARM_INDEX) rs01_rarm.set_zero_position_request = 1U;
        }
        else if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
            (arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm)[joint == ARM_JOINT_SHOULDER ? 0 : 1].set_zero_position_request = 1U;
        else if (joint == ARM_JOINT_WRIST)
            (arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm)->set_zero_position_request = 1U;
        else if (joint == ARM_JOINT_ROTATION)
            (arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm)->set_zero_position_request = 1U;
    }
    zero_calibration_previous[arm][joint] = active;
}

/* 模块实现细节。 */
static void Upper_ZeroCalibrationComplete(unsigned arm, unsigned joint)
{
    bool at_zero = false;
    bool pending = false;
    if (joint == ARM_JOINT_ROOT)
    {
        if (arm == ARM_RARM_INDEX) { pending = rs01_rarm.set_zero_position_request != 0U; at_zero = rs01_rarm.feedback_valid && fabsf(rs01_rarm.single_position) <= RS01_ZERO_POSITION_TOLERANCE_RAD; }
    }
    else if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
    {
        rs00_t *m = (arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm) + (joint == ARM_JOINT_SHOULDER ? 0 : 1);
        pending = m->set_zero_position_request != 0U; at_zero = m->feedback_valid && fabsf(m->single_position) <= RS00_ZERO_POSITION_TOLERANCE_RAD;
    }
    else if (joint == ARM_JOINT_WRIST)
    {
        j4310_t *m = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
        pending = m->set_zero_position_request != 0U; at_zero = m->feedback.received && fabsf(m->feedback.position_rad) <= J4310_ZERO_POSITION_TOLERANCE_RAD;
    }
    else if (joint == ARM_JOINT_ROTATION)
    {
        rs05_t *m = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
        pending = m->set_zero_position_request != 0U; at_zero = m->feedback_valid && fabsf(m->single_position) <= RS05_ZERO_POSITION_TOLERANCE_RAD;
    }
    if (arm_joint_zero_calibration[arm][joint] != 0U && !pending && at_zero)
    {
        arm_joint_zero_calibration[arm][joint] = 0U;
        zero_calibration_previous[arm][joint] = 0U;
    }
}
/* 模块实现细节。 */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 模块实现细节。 */
static void telemetry(upper_board_t *board, unsigned joint, float p, float v, float t, float temp, unsigned fault,
                      uint32_t now)
{
    board->joint_mrad[joint] = (int16_t)clampf(p * 1000.0f, -32768.0f, 32767.0f);
    board->speed_centi_rad_s[joint] = (int16_t)clampf(v * 100.0f, -32768.0f, 32767.0f);
    board->torque_centi_nm[joint] = (int16_t)clampf(t * 100.0f, -32768.0f, 32767.0f);
    board->temperature_c[joint] = (int16_t)clampf(temp, -32768.0f, 32767.0f);
    board->fault[joint] = (uint16_t)fault;
    board->last_tick = now;
    board->frame_count++;
}

/* Override gains after M2006_Init; retain each 鐢垫満's PID 鐘舵€?and limits. */
static void Upper_ConfigurePidGains(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* 模块实现细节。 */
static void Upper_ConfigureM2006Gains(void)
{
    Upper_ConfigurePidGains(&m2006_dcup[0].position_pid, BR_M2006_CUP0_POSITION_KP, BR_M2006_CUP0_POSITION_KI, BR_M2006_CUP0_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_dcup[0].speed_pid, BR_M2006_CUP0_SPEED_KP, BR_M2006_CUP0_SPEED_KI, BR_M2006_CUP0_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_ucup[0].position_pid, BR_M2006_CUP0_POSITION_KP, BR_M2006_CUP0_POSITION_KI, BR_M2006_CUP0_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_ucup[0].speed_pid, BR_M2006_CUP0_SPEED_KP, BR_M2006_CUP0_SPEED_KI, BR_M2006_CUP0_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_dcup[1].position_pid, BR_M2006_CUP1_POSITION_KP, BR_M2006_CUP1_POSITION_KI, BR_M2006_CUP1_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_dcup[1].speed_pid, BR_M2006_CUP1_SPEED_KP, BR_M2006_CUP1_SPEED_KI, BR_M2006_CUP1_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_ucup[1].position_pid, BR_M2006_CUP1_POSITION_KP, BR_M2006_CUP1_POSITION_KI, BR_M2006_CUP1_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_ucup[1].speed_pid, BR_M2006_CUP1_SPEED_KP, BR_M2006_CUP1_SPEED_KI, BR_M2006_CUP1_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_larm.position_pid, br_root_m2006_position_kp, br_root_m2006_position_ki, br_root_m2006_position_kd);
    Upper_ConfigurePidGains(&m2006_larm.speed_pid, br_root_m2006_speed_kp, br_root_m2006_speed_ki, br_root_m2006_speed_kd);
}

/* 模块实现细节。 */
void Upper_BindMotors(upper_t *upper)
{
    if (upper == NULL)
        return;
    /* 模块实现细节。 */
    m2006_larm.trajectory_time_s = BR_ROOT_M2006_TRAJECTORY_TIME_S;
    rs01_rarm.trajectory_time_s = BR_ROOT_RS01_TRAJECTORY_TIME_S;
    rs00_larm[0].trajectory_time_s = BR_BIG_ARM_RS00_TRAJECTORY_TIME_S;
    rs00_rarm[0].trajectory_time_s = BR_BIG_ARM_RS00_TRAJECTORY_TIME_S;
    rs00_larm[1].trajectory_time_s = BR_SMALL_ARM_RS00_TRAJECTORY_TIME_S;
    rs00_rarm[1].trajectory_time_s = BR_SMALL_ARM_RS00_TRAJECTORY_TIME_S;
    j4310_larm.trajectory_time_s = BR_J4310_TRAJECTORY_TIME_S;
    j4310_rarm.trajectory_time_s = BR_J4310_TRAJECTORY_TIME_S;
    rs05_larm.trajectory_time_s = BR_RS05_TRAJECTORY_TIME_S;
    rs05_rarm.trajectory_time_s = BR_RS05_TRAJECTORY_TIME_S;
    m3508_dcup[0].trajectory_time_s = BR_AUX_M3508_TRAJECTORY_TIME_S;
    m2006_dcup[0].trajectory_time_s = BR_M2006_CUP0_TRAJECTORY_TIME_S;
    m2006_ucup[0].trajectory_time_s = BR_M2006_CUP0_TRAJECTORY_TIME_S;
    m2006_dcup[1].trajectory_time_s = BR_M2006_CUP1_TRAJECTORY_TIME_S;
    m2006_ucup[1].trajectory_time_s = BR_M2006_CUP1_TRAJECTORY_TIME_S;
    Upper_ConfigureM2006Gains();
    upper->arm[0] = (arm_group_t){&rs05_larm, &j4310_larm, &rs00_larm[1], &rs00_larm[0], &m2006_larm, NULL};
    upper->arm[1] = (arm_group_t){&rs05_rarm, &j4310_rarm, &rs00_rarm[1], &rs00_rarm[0], NULL, &rs01_rarm};
    last_service_tick = 0U;
    memset(zero_calibration_previous, 0, sizeof(zero_calibration_previous));
}

/* 模块实现细节。 */
void Upper_OnFrame(upper_t *upper, const can_frame_t *frame)
{
    uint32_t now = HAL_GetTick();
    unsigned a;
    if (upper == NULL || frame == NULL)
        return;
    for (a = 0U; a < 2U; a++)
    {
        rs00_t *rs00 = a == 0U ? rs00_larm : rs00_rarm;
        rs05_t *rs05 = a == 0U ? &rs05_larm : &rs05_rarm;
        j4310_t *j4310 = a == 0U ? &j4310_larm : &j4310_rarm;
        if (RS00_OnFrame(&rs00[0], frame)) { arm_feedback_angle[a][ARM_JOINT_SHOULDER] = rs00[0].position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_SHOULDER, rs00[0].position, rs00[0].velocity, rs00[0].torque, 0, rs00[0].fault, now); return; }
        if (RS00_OnFrame(&rs00[1], frame)) { arm_feedback_angle[a][ARM_JOINT_ELBOW] = rs00[1].position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_ELBOW, rs00[1].position, rs00[1].velocity, rs00[1].torque, 0, rs00[1].fault, now); return; }
        if (J4310_OnFrame(j4310, frame)) { arm_feedback_angle[a][ARM_JOINT_WRIST] = j4310->feedback.position_rad * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_WRIST, j4310->feedback.position_rad, j4310->feedback.speed_rad_s, j4310->feedback.torque_nm, j4310->feedback.mos_temperature, j4310->feedback.state, now); return; }
        if (RS05_OnFrame(rs05, frame)) { arm_feedback_angle[a][ARM_JOINT_ROTATION] = rs05->position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_ROTATION, rs05->position, rs05->velocity, rs05->torque, 0, rs05->fault, now); return; }
    }
    if (M2006_OnFrame(&m2006_larm, frame))
    {
        arm_feedback_angle[ARM_LARM_INDEX][ARM_JOINT_ROOT] = m2006_larm.position_rad * ARM_RAD_TO_DEG;
        telemetry(&upper->board[0], ARM_JOINT_ROOT, m2006_larm.position_rad, m2006_larm.speed_rad_s,
                  m2006_larm.current_raw * M2006_CURRENT_SCALE * BR_LEFT_ROOT_NM_PER_A, 0, 0, now);
        return;
    }
    if (RS01_OnFrame(&rs01_rarm, frame))
    {
        arm_feedback_angle[ARM_RARM_INDEX][ARM_JOINT_ROOT] = rs01_rarm.position * ARM_RAD_TO_DEG;
        telemetry(&upper->board[1], ARM_JOINT_ROOT, rs01_rarm.position, rs01_rarm.velocity, rs01_rarm.torque, 0,
                  rs01_rarm.fault, now);
    }
}

/* 模块实现细节。 */
static rs_mode_t Upper_ToRsMode(arm_mode_t mode)
{
    switch (mode)
    {
    case ARM_MODE_DAMPING: return RS_MODE_DAMPING;
    case ARM_MODE_SPEED: return RS_MODE_SPEED;
    case ARM_MODE_POSITION: return RS_MODE_POSITION;
    case ARM_MODE_MIT: return RS_MODE_MIT;
    case ARM_MODE_STOP:
    default: return RS_MODE_STOP;
    }
}

/* 模块实现细节。 */
static j4310_mode_t Upper_ToJ4310Mode(arm_mode_t mode)
{
    switch (mode)
    {
    case ARM_MODE_DAMPING: return J4310_MODE_DAMPING;
    case ARM_MODE_SPEED: return J4310_MODE_SPEED;
    case ARM_MODE_POSITION: return J4310_MODE_POSITION;
    case ARM_MODE_MIT: return J4310_MODE_MIT;
    case ARM_MODE_STOP:
    default: return J4310_MODE_STOP;
    }
}

/* 模块实现细节。 */
static void apply_arm(void *motor, unsigned arm, arm_joint_t joint, float tau)
{
    arm_mode_t mode = arm_mode[arm][joint];
    float target = arm_target[arm][joint]; /* 模块实现细节。 */
    /* 模块实现细节。 */
    if (joint == ARM_JOINT_ROOT)
    {
        tau = mode == ARM_MODE_MIT ? br_mit_tau[arm][joint] : 0.0f;
        if (arm == ARM_LARM_INDEX)
        {
            m2006_t *root = (m2006_t *)motor;
            /* 模块实现细节。 */
            /* Keep the unified target array, but always run the motor in its
             position-speed cascade. */
            (void)mode;
            (void)tau;
            M2006_SetTargetDeferred(root, M2006_MODE_POSITION,
                                    (target * ARM_DEG_TO_RAD - BR_LEFT_ROOT_START_RAD) /
                                        BR_LEFT_ROOT_JOINT_PER_OUTPUT * ARM_RAD_TO_DEG);
        }
        else if (arm == ARM_RARM_INDEX)
        {
            rs01_t *root = (rs01_t *)motor;
            if (mode == ARM_MODE_MIT)
                RS01_SetTargetMIT(root, RS_MODE_MIT, tau, target);
            else
                RS01_SetTargetBasic(root, Upper_ToRsMode(mode), target);
        }
        return;
    }

    if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
    {
        rs00_t *m = (rs00_t *)motor;
        m->tau_ff_nm = tau;
        if (mode == ARM_MODE_MIT) RS00_SetTargetMIT(m, RS_MODE_MIT, tau, target);
/* 模块实现细节。 */
        else RS00_SetTargetBasic(m, Upper_ToRsMode(mode), target);
    }
    else if (joint == ARM_JOINT_WRIST)
    {
        j4310_t *m = (j4310_t *)motor;
        if (mode == ARM_MODE_MIT) J4310_SetTargetMIT(m, J4310_MODE_MIT, tau, target);
/* 模块实现细节。 */
        else J4310_SetTargetBasic(m, Upper_ToJ4310Mode(mode), target);
    }
    else if (joint == ARM_JOINT_ROTATION)
    {
        rs05_t *m = (rs05_t *)motor;
        m->tau_ff_nm = tau;
        if (mode == ARM_MODE_MIT) RS05_SetTargetMIT(m, RS_MODE_MIT, tau, target);
/* 模块实现细节。 */
        else RS05_SetTargetBasic(m, Upper_ToRsMode(mode), target);
    }
}
/* 模块实现细节。 */
void Upper_Service(uint32_t now)
{
    float dt = last_service_tick == 0U ? 0.002f : (now - last_service_tick) * 0.001f;
    unsigned a;
    if (dt <= 0.0f || dt > 0.02f) dt = 0.002f;
    last_service_tick = now;
    configure_motor_gains();
    Upper_ConfigureM2006Gains();
    for (a = 0U; a < 2U; a++)
        for (unsigned zero_joint = 0U; zero_joint < ARM_JOINT_COUNT; zero_joint++)
            Upper_ZeroCalibrationRequest(a, zero_joint);
    for (a = 0U; a < 2U; a++)
    {
        bool enabled = upper_state.enable != 0U;
        float q[BR_ARM_DOF] = {0}, q_deg[BR_ARM_DOF] = {0}, tau[BR_ARM_DOF] = {0};
        rs00_t *rs00 = a == 0U ? rs00_larm : rs00_rarm;
        rs05_t *rs05 = a == 0U ? &rs05_larm : &rs05_rarm;
        j4310_t *j4310 = a == 0U ? &j4310_larm : &j4310_rarm;
        bool online = rs00[0].feedback_valid && rs00[1].feedback_valid && rs05->feedback_valid && j4310->feedback.received;
        q_deg[0] = a == 0U ? m2006_larm.position_rad * ARM_RAD_TO_DEG : rs01_rarm.position * ARM_RAD_TO_DEG;
        q_deg[1] = rs00[0].position * ARM_RAD_TO_DEG;
        q_deg[2] = rs00[1].position * ARM_RAD_TO_DEG;
        q_deg[3] = j4310->feedback.position_rad * ARM_RAD_TO_DEG;
        q_deg[4] = rs05->position * ARM_RAD_TO_DEG;
        for (unsigned joint = 0U; joint < BR_ARM_DOF; joint++)
            q[joint] = q_deg[joint] * ARM_DEG_TO_RAD;
        upper_state.board[a].online = online;
        upper_state.board[a].gravity_ready = online && BR_GravityEvaluate(&gravity_model[a], q, br_grip[a], tau, NULL);
        if (!upper_state.board[a].gravity_ready) memset(tau, 0, sizeof(tau));
        if (br_manual_tau_enable != 0U)
        {
            /* 模块实现细节。 */
            tau[ARM_JOINT_SHOULDER] = br_shoulder_tau_nm;
            tau[ARM_JOINT_ELBOW] = br_elbow_tau_nm;
            tau[ARM_JOINT_WRIST] = br_wrist_tau_nm;
            tau[ARM_JOINT_ROTATION] = br_rotation_tau_nm;
        }
        if (!enabled)
        {
            RS00_SetTargetBasic(&rs00[0], RS_MODE_STOP, 0); RS00_SetTargetBasic(&rs00[1], RS_MODE_STOP, 0);
            RS05_SetTargetBasic(rs05, RS_MODE_STOP, 0); J4310_SetTargetBasic(j4310, J4310_MODE_STOP, 0);
            if (a == 1U) RS01_SetTargetBasic(&rs01_rarm, RS_MODE_STOP, 0);
/* 模块实现细节。 */
            else M2006_SetTargetDeferred(&m2006_larm, M2006_MODE_STOP, 0);
        }
        else
        {
            apply_arm(a == ARM_LARM_INDEX ? (void *)&m2006_larm : (void *)&rs01_rarm,
                      a, ARM_JOINT_ROOT, 0.0f);
            apply_arm(&rs00[0], a, ARM_JOINT_SHOULDER, tau[1]);
            apply_arm(&rs00[1], a, ARM_JOINT_ELBOW, tau[2]);
            apply_arm(j4310, a, ARM_JOINT_WRIST, tau[3]);
            apply_arm(rs05, a, ARM_JOINT_ROTATION, tau[4]);
        }
        RS00_Service(&rs00[0]); RS00_Service(&rs00[1]); RS05_Service(rs05); J4310_Service(j4310, tau[3], now);
        if (a == 1U) RS01_Service(&rs01_rarm);
        memcpy((void *)upper_state.board[a].gravity_tau_nm, tau, sizeof(tau));
        for (unsigned zero_joint = 0U; zero_joint < ARM_JOINT_COUNT; zero_joint++)
            Upper_ZeroCalibrationComplete(a, zero_joint);
    }
}
