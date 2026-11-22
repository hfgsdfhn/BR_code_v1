
#include "upper.h"
#include "cup_config.h"
#include "cup_task.h"
#include "arm_task.h"
#include "my_main.h"
#include <math.h>
#include <string.h>

extern volatile float arm_target[2][ARM_JOINT_COUNT];
extern volatile float arm_pose_pd11[2][ARM_JOINT_COUNT];
extern volatile float br_mit_tau[2][ARM_JOINT_COUNT];
extern float br_joint_velocity[2][ARM_JOINT_COUNT];
extern bool br_grip[2];
extern volatile uint8_t arm_joint_zero_calibration[2][ARM_JOINT_COUNT];
extern volatile bool startup_hold_latched[2];
static uint8_t zero_calibration_previous[2][ARM_JOINT_COUNT];

__ALIGNED(32) volatile float arm_feedback_angle[2][ARM_JOINT_COUNT];
volatile float arm_end_position_world[2][3];
volatile float arm_end_target_world[2][3];
volatile uint8_t arm_ik_enable[2] = {0U, 0U};
volatile uint8_t arm_ik_second_rs00_prefer_small[2];
upper_t upper_state;
Arm_Pos Arm_larm, Arm_rarm;

volatile float br_root_m2006_trajectory_time_s = 2.0f;
volatile float br_root_rs01_trajectory_time_s = 0.5f;
volatile float br_big_arm_rs00_trajectory_time_s = 1.4f;
volatile float br_small_arm_rs00_trajectory_time_s = 1.0f;
volatile float br_j4310_trajectory_time_s = 0.5f;
volatile float br_rs05_trajectory_time_s = 0.0f;

volatile float br_root_rs01_kp = 200.0f;
volatile float br_root_rs01_kd = 50.0f;
volatile float br_shoulder_kp = 50.0f;
volatile float br_shoulder_kd = 5.0f;
volatile float br_elbow_kp = 50.0f;
volatile float br_elbow_kd = 3.0f;
volatile float br_wrist_kp = 30.0f;
volatile float br_wrist_kd = 2.0f;
volatile float br_rotation_kp = 30.0f;
volatile float br_rotation_kd = 5.0f;
volatile float br_root_m2006_position_kp = 100.247779608f;
volatile float br_root_m2006_position_ki = 60.359877560f;
volatile float br_root_m2006_position_kd = 0.0f;
volatile float br_root_m2006_speed_kp = 3.0f;
volatile float br_root_m2006_speed_ki = 3.0f;
volatile float br_root_m2006_speed_kd = 0.0f;

volatile float br_m2006_cup0_position_kp = BR_M2006_CUP0_POSITION_KP;
volatile float br_m2006_cup0_position_ki = BR_M2006_CUP0_POSITION_KI;
volatile float br_m2006_cup0_position_kd = BR_M2006_CUP0_POSITION_KD;
volatile float br_m2006_cup0_speed_kp = BR_M2006_CUP0_SPEED_KP;
volatile float br_m2006_cup0_speed_ki = BR_M2006_CUP0_SPEED_KI;
volatile float br_m2006_cup0_speed_kd = BR_M2006_CUP0_SPEED_KD;
volatile float br_m2006_cup1_position_kp = BR_M2006_CUP1_POSITION_KP;
volatile float br_m2006_cup1_position_ki = BR_M2006_CUP1_POSITION_KI;
volatile float br_m2006_cup1_position_kd = BR_M2006_CUP1_POSITION_KD;
volatile float br_m2006_cup1_speed_kp = BR_M2006_CUP1_SPEED_KP;
volatile float br_m2006_cup1_speed_ki = BR_M2006_CUP1_SPEED_KI;
volatile float br_m2006_cup1_speed_kd = BR_M2006_CUP1_SPEED_KD;

volatile uint8_t g_rs_recovery_motion_release = 0U;

volatile uint8_t larm_manual_tau_enable = 0U;
volatile float larm_shoulder_tau_nm = 0.0f;
volatile float larm_elbow_tau_nm = 0.0f;
volatile float larm_wrist_tau_nm = 0.0f;
volatile float larm_rotation_tau_nm = 0.0f;
volatile uint8_t rarm_manual_tau_enable = 0U;
volatile float rarm_shoulder_tau_nm = 0.0f;
volatile float rarm_elbow_tau_nm = 0.0f;
volatile float rarm_wrist_tau_nm = 0.0f;
volatile float rarm_rotation_tau_nm = 0.0f;
volatile float larm_mass_003_kg = 1.5f;
volatile float larm_mass_004_kg = 0.001f;
volatile float larm_mass_005_kg = 0.06f;
volatile float larm_mass_006_kg = 1.22f;
volatile float rarm_mass_003_kg = 1.35f;
volatile float rarm_mass_004_kg = 0.1f;
volatile float rarm_mass_005_kg = 0.3f;
volatile float rarm_mass_006_kg = 1.2f;


static void Upper_ApplyTrajectoryTimes(void)
{
    m2006_larm.trajectory_time_s = br_root_m2006_trajectory_time_s;
    rs01_rarm.trajectory_time_s = br_root_rs01_trajectory_time_s;
    rs00_larm[0].trajectory_time_s = br_big_arm_rs00_trajectory_time_s;
    rs00_rarm[0].trajectory_time_s = br_big_arm_rs00_trajectory_time_s;
    rs00_larm[1].trajectory_time_s = br_small_arm_rs00_trajectory_time_s;
    rs00_rarm[1].trajectory_time_s = br_small_arm_rs00_trajectory_time_s;
    j4310_larm.trajectory_time_s = br_j4310_trajectory_time_s;
    j4310_rarm.trajectory_time_s = br_j4310_trajectory_time_s;
    rs05_larm.trajectory_time_s = br_rs05_trajectory_time_s;
    rs05_rarm.trajectory_time_s = br_rs05_trajectory_time_s;
}


void Upper_SetArmIKTarget(uint8_t arm, float x_cm, float y_cm, float z_cm)
{
    if (arm >= 2U || !isfinite(x_cm) || !isfinite(y_cm) || !isfinite(z_cm))
        return;
    arm_end_target_world[arm][0] = x_cm;
    arm_end_target_world[arm][1] = y_cm <= 10.0f ? 10.1f : y_cm;
    arm_end_target_world[arm][2] = z_cm;
}

void Upper_SetArmIKEnabled(uint8_t arm, bool enable)
{
    if (arm < 2U)
        arm_ik_enable[arm] = enable ? 1U : 0U;
}
void Upper_RefreshArmIK(uint8_t arm)
{
    float q[BR_ARM_DOF] = {0.0f};
    float target_world_m[3];
    float target_deg[BR_ARM_DOF];

    if (arm >= 2U)
        return;
    for (unsigned joint = 0U; joint < BR_ARM_DOF; ++joint)
        q[joint] = arm_feedback_angle[arm][joint] * ARM_DEG_TO_RAD;
    target_world_m[0] = arm_end_target_world[arm][0] * 0.01f;
    target_world_m[1] = arm_end_target_world[arm][1] * 0.01f;
    target_world_m[2] = arm_end_target_world[arm][2] * 0.01f;
    if (BR_InverseKinematics(&br_gravity_models[arm], target_world_m, q,
                             target_deg,
                             arm_ik_second_rs00_prefer_small[arm] != 0U))
    {
        for (unsigned joint = ARM_JOINT_ROOT; joint <= ARM_JOINT_WRIST; ++joint)
            arm_target[arm][joint] = target_deg[joint];
    }
}
static uint32_t last_service_tick;
static volatile bool arm_overtemp_latched[2];


bool apply_arm(uint8_t arm, uint8_t joint_index, float target_degrees)
{
    if (joint_index < 1U || joint_index > ARM_JOINT_COUNT)
    {
        return false;
    }
    return Upper_SetJointDegrees(arm,
                                 (arm_joint_t)(joint_index - 1U),
                                 target_degrees);
}

bool apply_cup(uint8_t cup, uint8_t index, float target_degrees)
{
    if (cup >= 2U || index >= 3U)
    {
        return false;
    }
    suction_target[cup][index] = target_degrees;
    return true;
}


static void Upper_ZeroCalibrationRequest(unsigned arm, unsigned joint)
{
    uint8_t active = arm_joint_zero_calibration[arm][joint] != 0U;

    if (active)
    {
        if (joint == ARM_JOINT_ROOT && arm == ARM_RARM_INDEX)
        {
            if (!zero_calibration_previous[arm][joint])
                RS01_ResetSoftwareZero(&rs01_rarm);
        }
        else if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
            (arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm)[joint == ARM_JOINT_SHOULDER ? 0 : 1].set_zero_position_request = 1U;
        else if (joint == ARM_JOINT_WRIST)
            (arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm)->set_zero_position_request = 1U;
        else if (joint == ARM_JOINT_ROTATION)
        {
            rs05_t *m = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
            if (!zero_calibration_previous[arm][joint])
                RS05_ResetSoftwareZero(m);
        }
    }
    zero_calibration_previous[arm][joint] = active;
}

static void Upper_ZeroCalibrationComplete(unsigned arm, unsigned joint)
{
    bool at_zero = false;
    bool pending = false;
    if (joint == ARM_JOINT_ROOT)
    {
        if (arm == ARM_RARM_INDEX){pending = false; at_zero = rs01_rarm.feedback_valid && fabsf(rs01_rarm.position) <= RS01_ZERO_POSITION_TOLERANCE_RAD;}
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
        pending = false; at_zero = m->feedback_valid && fabsf(m->position) <= RS05_ZERO_POSITION_TOLERANCE_RAD;
    }
    if (arm_joint_zero_calibration[arm][joint] != 0U && !pending && at_zero)
    {
        arm_feedback_angle[arm][joint] = 0.0f;
        arm_target[arm][joint] = arm_pose_pd11[arm][joint];
        arm_joint_zero_calibration[arm][joint] = 0U;
        zero_calibration_previous[arm][joint] = 0U;
    }
}
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

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

static void Upper_ConfigurePidGains(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

static void Upper_ConfigureM2006Gains(void)
{
    Upper_ConfigurePidGains(&m2006_dcup[0].position_pid, br_m2006_cup0_position_kp, br_m2006_cup0_position_ki, br_m2006_cup0_position_kd);
    Upper_ConfigurePidGains(&m2006_dcup[0].speed_pid, br_m2006_cup0_speed_kp, br_m2006_cup0_speed_ki, br_m2006_cup0_speed_kd);
    Upper_ConfigurePidGains(&m2006_ucup[0].position_pid, br_m2006_cup0_position_kp, br_m2006_cup0_position_ki, br_m2006_cup0_position_kd);
    Upper_ConfigurePidGains(&m2006_ucup[0].speed_pid, br_m2006_cup0_speed_kp, br_m2006_cup0_speed_ki, br_m2006_cup0_speed_kd);
    Upper_ConfigurePidGains(&m2006_dcup[1].position_pid, br_m2006_cup1_position_kp, br_m2006_cup1_position_ki, br_m2006_cup1_position_kd);
    Upper_ConfigurePidGains(&m2006_dcup[1].speed_pid, br_m2006_cup1_speed_kp, br_m2006_cup1_speed_ki, br_m2006_cup1_speed_kd);
    Upper_ConfigurePidGains(&m2006_ucup[1].position_pid, br_m2006_cup1_position_kp, br_m2006_cup1_position_ki, br_m2006_cup1_position_kd);
    Upper_ConfigurePidGains(&m2006_ucup[1].speed_pid, br_m2006_cup1_speed_kp, br_m2006_cup1_speed_ki, br_m2006_cup1_speed_kd);
    Upper_ConfigurePidGains(&m2006_larm.position_pid, br_root_m2006_position_kp, br_root_m2006_position_ki, br_root_m2006_position_kd);
    Upper_ConfigurePidGains(&m2006_larm.speed_pid, br_root_m2006_speed_kp, br_root_m2006_speed_ki, br_root_m2006_speed_kd);
}

void Upper_BindMotors(upper_t *upper)
{
    if (upper == NULL)
        return;

    Upper_ApplyTrajectoryTimes();
    rs00_larm[0].damping_kd = 2.0f;
    rs00_larm[1].damping_kd = 4.0f;
    rs00_rarm[0].damping_kd = 2.0f;
    rs00_rarm[1].damping_kd = 4.0f;
    m3508_dcup[0].trajectory_time_s = BR_AUX_M3508_TRAJECTORY_TIME_S;
    m2006_dcup[0].trajectory_time_s = BR_M2006_CUP0_TRAJECTORY_TIME_S;
    m2006_ucup[0].trajectory_time_s = BR_M2006_CUP0_TRAJECTORY_TIME_S;
    m2006_dcup[0].trajectory_enabled = BR_M2006_CUP0_USE_QUINTIC != 0U;
    m2006_ucup[0].trajectory_enabled = BR_M2006_CUP0_USE_QUINTIC != 0U;
    m2006_dcup[1].trajectory_time_s = BR_M2006_CUP1_TRAJECTORY_TIME_S;
    m2006_ucup[1].trajectory_time_s = BR_M2006_CUP1_TRAJECTORY_TIME_S;
    m2006_dcup[1].trajectory_enabled = BR_M2006_CUP1_USE_QUINTIC != 0U;
    m2006_ucup[1].trajectory_enabled = BR_M2006_CUP1_USE_QUINTIC != 0U;
    Upper_ConfigureM2006Gains();
    upper->arm[0] = (arm_group_t){&rs05_larm, &j4310_larm, &rs00_larm[1], &rs00_larm[0], &m2006_larm, NULL};
    upper->arm[1] = (arm_group_t){&rs05_rarm, &j4310_rarm, &rs00_rarm[1], &rs00_rarm[0], NULL, &rs01_rarm};
    last_service_tick = 0U;
    memset(zero_calibration_previous, 0, sizeof(zero_calibration_previous));
}

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
        if (RS00_OnFrame(&rs00[0], frame)) { arm_feedback_angle[a][ARM_JOINT_SHOULDER] = rs00[0].position * ARM_RAD_TO_DEG * 20.0f / 36.0f; telemetry(&upper->board[a], ARM_JOINT_SHOULDER, rs00[0].position, rs00[0].velocity, rs00[0].torque, rs00[0].temperature_c, rs00[0].fault, now); return; }
        if (RS00_OnFrame(&rs00[1], frame)) { arm_feedback_angle[a][ARM_JOINT_ELBOW] = rs00[1].position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_ELBOW, rs00[1].position, rs00[1].velocity, rs00[1].torque, rs00[1].temperature_c, rs00[1].fault, now); return; }
        if (J4310_OnFrame(j4310, frame)) { arm_feedback_angle[a][ARM_JOINT_WRIST] = j4310->feedback.position_rad * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_WRIST, j4310->feedback.position_rad, j4310->feedback.speed_rad_s, j4310->feedback.torque_nm, j4310->feedback.mos_temperature, j4310->feedback.state, now); return; }
        if (RS05_OnFrame(rs05, frame)) { arm_feedback_angle[a][ARM_JOINT_ROTATION] = rs05->position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_ROTATION, rs05->position, rs05->velocity, rs05->torque, rs05->temperature_c, rs05->fault, now); return; }
    }
    if (M2006_OnFrame(&m2006_larm, frame))
    {
        arm_feedback_angle[ARM_LARM_INDEX][ARM_JOINT_ROOT] = m2006_larm.position_rad * ARM_RAD_TO_DEG * 14.0f / 45.0f;
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

static bool Upper_ArmTemperatureSafe(unsigned arm)
{
    rs00_t *rs00 = arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
    rs01_t *rs01 = &rs01_rarm;
    if (!j4310->feedback.received) return true;
    if (rs00[0].temperature_c >= ARM_OVERTEMP_CLEAR_TEMPERATURE_C ||
        rs00[1].temperature_c >= ARM_OVERTEMP_CLEAR_TEMPERATURE_C ||
        rs05->temperature_c >= ARM_OVERTEMP_CLEAR_TEMPERATURE_C ||
        j4310->feedback.mos_temperature >= ARM_OVERTEMP_CLEAR_TEMPERATURE_C ||
        j4310->feedback.rotor_temperature >= ARM_OVERTEMP_CLEAR_TEMPERATURE_C)
        return false;
    return arm == ARM_LARM_INDEX || rs01->temperature_c < ARM_OVERTEMP_CLEAR_TEMPERATURE_C;
}

static bool Upper_ArmTemperatureOverLimit(unsigned arm)
{
    rs00_t *rs00 = arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
    rs01_t *rs01 = &rs01_rarm;
    if (!j4310->feedback.received) return false;
    return rs00[0].temperature_c > ARM_OVERTEMP_TRIP_TEMPERATURE_C ||
           rs00[1].temperature_c > ARM_OVERTEMP_TRIP_TEMPERATURE_C ||
           rs05->temperature_c > ARM_OVERTEMP_TRIP_TEMPERATURE_C ||
           j4310->feedback.mos_temperature > ARM_OVERTEMP_TRIP_TEMPERATURE_C ||
           j4310->feedback.rotor_temperature > ARM_OVERTEMP_TRIP_TEMPERATURE_C ||
           (arm == ARM_RARM_INDEX && rs01->temperature_c > ARM_OVERTEMP_TRIP_TEMPERATURE_C);
}static void Upper_ApplyRecoveryState(unsigned arm, bool active)
{
    rs00_t *rs00 = arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
    rs00[0].recovery_trajectory = active;
    rs00[1].recovery_trajectory = active;
    rs05->recovery_trajectory = active;
    j4310->recovery_trajectory = active;
    if (arm == ARM_LARM_INDEX) m2006_larm.recovery_trajectory = active;
    else rs01_rarm.recovery_trajectory = active;
    if (active)
    {
        for (unsigned joint = 0U; joint < ARM_JOINT_COUNT; joint++)
        {
            arm_mode[arm][joint] = ARM_MODE_MIT;
            arm_target[arm][joint] = arm_pose_pd11[arm][joint];
        }
    }
}

bool Upper_IsArmOvertemperature(uint8_t arm)
{
    return arm < 2U && arm_overtemp_latched[arm];
}
static void apply_arm_motor(void *motor, unsigned arm, arm_joint_t joint, float tau)
{
    arm_mode_t mode = arm_mode[arm][joint];
    float target = arm_target[arm][joint];

    if (joint == ARM_JOINT_ROOT)
    {
        tau = mode == ARM_MODE_MIT ? br_mit_tau[arm][joint] : 0.0f;
        if (arm == ARM_LARM_INDEX)
        {
            m2006_t *root_motor = (m2006_t *)motor;
            m2006_t *root = (m2006_t *)motor;
            (void)mode;
            (void)tau;
            M2006_SetTargetDeferred(root_motor, M2006_MODE_POSITION,
                                    ((target * 45.0f / 14.0f) * ARM_DEG_TO_RAD - BR_LEFT_ROOT_START_RAD) /
                                        BR_LEFT_ROOT_JOINT_PER_OUTPUT * ARM_RAD_TO_DEG);
        }
        else if (arm == ARM_RARM_INDEX)
        {
            rs01_t *root_motor = (rs01_t *)motor;
            if (mode == ARM_MODE_MIT)
                RS01_SetTargetMIT(root_motor, RS_MODE_MIT, tau, target);
            else
                RS01_SetTargetBasic(root_motor, Upper_ToRsMode(mode), target);
        }
        return;
    }

    if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
    {
        rs00_t *m = (rs00_t *)motor;
        if (joint == ARM_JOINT_SHOULDER)
        {
            target *= 36.0f / 20.0f;
            tau *= 20.0f / 36.0f;
        }
        m->tau_ff_nm = tau;
        if (mode == ARM_MODE_MIT) RS00_SetTargetMIT(m, RS_MODE_MIT, tau, target);

        else RS00_SetTargetBasic(m, Upper_ToRsMode(mode), target);
    }
    else if (joint == ARM_JOINT_WRIST)
    {
        j4310_t *m = (j4310_t *)motor;
        if (mode == ARM_MODE_MIT) J4310_SetTargetMIT(m, J4310_MODE_MIT, tau, target);

        else J4310_SetTargetBasic(m, Upper_ToJ4310Mode(mode), target);
    }
    else if (joint == ARM_JOINT_ROTATION)
    {
        rs05_t *m = (rs05_t *)motor;
        m->tau_ff_nm = tau;
        if (mode == ARM_MODE_MIT) RS05_SetTargetMIT(m, RS_MODE_MIT, tau, target);

        else RS05_SetTargetBasic(m, Upper_ToRsMode(mode), target);
    }
}
void Upper_Service(uint32_t now) {
  Upper_ApplyTrajectoryTimes();
  float dt =
      last_service_tick == 0U ? 0.002f : (now - last_service_tick) * 0.001f;
  unsigned a;
  if (dt <= 0.0f || dt > 0.02f)
    dt = 0.002f;
  last_service_tick = now;
  configure_motor_gains();
  for (a = 0U; a < 2U; a++) {
    if (ARM_OVERTEMP_PROTECTION_ENABLED != 0U) {
      if (arm_overtemp_latched[a] && Upper_ArmTemperatureSafe(a))
        arm_overtemp_latched[a] = false;
      else if (!arm_overtemp_latched[a] && Upper_ArmTemperatureOverLimit(a))
        arm_overtemp_latched[a] = true;
    } else {
      arm_overtemp_latched[a] = false;
    }
    Upper_ApplyRecoveryState(a, arm_overtemp_latched[a]);
  }
  Upper_ConfigureM2006Gains();
  for (a = 0U; a < 2U; a++)
    for (unsigned zero_joint = 0U; zero_joint < ARM_JOINT_COUNT; zero_joint++)
      Upper_ZeroCalibrationRequest(a, zero_joint);
  {
    bool requested_enabled = upper_state.enable != 0U;
    for (a = 0U; a < 2U; a++) {
    bool enabled = requested_enabled;
    float q[BR_ARM_DOF] = {0}, q_deg[BR_ARM_DOF] = {0}, tau[BR_ARM_DOF] = {0};
    rs00_t *rs00 = a == 0U ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = a == 0U ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = a == 0U ? &j4310_larm : &j4310_rarm;
    bool online = rs00[0].feedback_valid && rs00[1].feedback_valid &&
                  rs05->feedback_valid && j4310->feedback.received;
    if (!startup_hold_latched[a] && online)
    {
      for (unsigned joint = 0U; joint < ARM_JOINT_COUNT; ++joint)
      {
        arm_target[a][joint] = arm_feedback_angle[a][joint];
        arm_mode[a][joint] = ARM_MODE_MIT;
      }
      startup_hold_latched[a] = true;
    }

    q_deg[0] = arm_feedback_angle[a][ARM_JOINT_ROOT];
    q_deg[1] = arm_feedback_angle[a][ARM_JOINT_SHOULDER];
    q_deg[2] = arm_feedback_angle[a][ARM_JOINT_ELBOW];
    q_deg[3] = arm_feedback_angle[a][ARM_JOINT_WRIST];
    q_deg[4] = arm_feedback_angle[a][ARM_JOINT_ROTATION];
    for (unsigned joint = 0U; joint < BR_ARM_DOF; joint++)
      q[joint] = q_deg[joint] * ARM_DEG_TO_RAD;
    if (arm_ik_enable[a] != 0U && online)
    {
      float target_world_m[3] = {
          arm_end_target_world[a][0] * 0.01f,
          arm_end_target_world[a][1] * 0.01f,
          arm_end_target_world[a][2] * 0.01f};
      float ik_target_deg[BR_ARM_DOF];
      if (BR_InverseKinematics(&br_gravity_models[a], target_world_m, q,
                               ik_target_deg,
                               arm_ik_second_rs00_prefer_small[a] != 0U))
      {
        for (unsigned joint = ARM_JOINT_ROOT; joint <= ARM_JOINT_WRIST; ++joint)
          arm_target[a][joint] = ik_target_deg[joint];
      }
    }
    if (arm_overtemp_latched[a])
      Upper_ApplyRecoveryState(a, true);
    upper_state.board[a].online = online;
    br_gravity_models[a].payload_mass_kg = 0.174f;
    if (a == ARM_LARM_INDEX)
    {
      br_gravity_models[a].link_mass_kg[0] = larm_mass_003_kg;
      br_gravity_models[a].link_mass_kg[1] = larm_mass_004_kg;
      br_gravity_models[a].link_mass_kg[2] = larm_mass_005_kg;
      br_gravity_models[a].link_mass_kg[3] = larm_mass_006_kg;
    }
    else
    {
      br_gravity_models[a].link_mass_kg[0] = rarm_mass_003_kg;
      br_gravity_models[a].link_mass_kg[1] = rarm_mass_004_kg;
      br_gravity_models[a].link_mass_kg[2] = rarm_mass_005_kg;
      br_gravity_models[a].link_mass_kg[3] = rarm_mass_006_kg;
    }
    /* Payload compensation stays off regardless of the PD5 and PD6 outputs. */
    upper_state.board[a].gravity_ready =
        online && BR_GravityEvaluate(&br_gravity_models[a], q, false, tau, NULL);
    float end_position_m[3];
    if (!BR_ForwardKinematics(&br_gravity_models[a], q, end_position_m)) {
      arm_end_position_world[a][0] = 0.0f;
      arm_end_position_world[a][1] = 0.0f;
      arm_end_position_world[a][2] = 0.0f;
    } else {
      arm_end_position_world[a][0] = end_position_m[0] * 100.0f;
      arm_end_position_world[a][1] = end_position_m[1] * 100.0f;
      arm_end_position_world[a][2] = end_position_m[2] * 100.0f;
    }
    if (!upper_state.board[a].gravity_ready)
      memset(tau, 0, sizeof(tau));
    if (a == ARM_LARM_INDEX)
    {
      if (larm_manual_tau_enable == 0U)
      {
        larm_shoulder_tau_nm = tau[ARM_JOINT_SHOULDER];
        larm_elbow_tau_nm = tau[ARM_JOINT_ELBOW];
        larm_wrist_tau_nm = tau[ARM_JOINT_WRIST];
        larm_rotation_tau_nm = tau[ARM_JOINT_ROTATION];
      }
      else
      {
        tau[ARM_JOINT_SHOULDER] = larm_shoulder_tau_nm;
        tau[ARM_JOINT_ELBOW] = larm_elbow_tau_nm;
        tau[ARM_JOINT_WRIST] = larm_wrist_tau_nm;
        tau[ARM_JOINT_ROTATION] = larm_rotation_tau_nm;
      }
    }
    else
    {
      if (rarm_manual_tau_enable == 0U)
      {
        rarm_shoulder_tau_nm = tau[ARM_JOINT_SHOULDER];
        rarm_elbow_tau_nm = tau[ARM_JOINT_ELBOW];
        rarm_wrist_tau_nm = tau[ARM_JOINT_WRIST];
        rarm_rotation_tau_nm = tau[ARM_JOINT_ROTATION];
      }
      else
      {
        tau[ARM_JOINT_SHOULDER] = rarm_shoulder_tau_nm;
        tau[ARM_JOINT_ELBOW] = rarm_elbow_tau_nm;
        tau[ARM_JOINT_WRIST] = rarm_wrist_tau_nm;
        tau[ARM_JOINT_ROTATION] = rarm_rotation_tau_nm;
      }
    }
    if (!enabled) {
      memset(tau, 0, sizeof(tau));
      RS00_SetTargetBasic(&rs00[0], RS_MODE_DAMPING, 0);
      RS00_SetTargetBasic(&rs00[1], RS_MODE_DAMPING, 0);
      RS05_SetTargetBasic(rs05, RS_MODE_DAMPING, 0);
      J4310_SetTargetBasic(j4310, J4310_MODE_DAMPING, 0);
      if (a == 1U)
        RS01_SetTargetBasic(&rs01_rarm, RS_MODE_DAMPING, 0);
      else
        M2006_SetTargetDeferred(&m2006_larm, M2006_MODE_DAMPING, 0);
    } else {
      apply_arm_motor(a == ARM_LARM_INDEX ? (void *)&m2006_larm
                                          : (void *)&rs01_rarm,
                      a, ARM_JOINT_ROOT, 0.0f);
      apply_arm_motor(&rs00[0], a, ARM_JOINT_SHOULDER, tau[1]);
      apply_arm_motor(&rs00[1], a, ARM_JOINT_ELBOW, tau[2]);
      apply_arm_motor(j4310, a, ARM_JOINT_WRIST, tau[3]);
      apply_arm_motor(rs05, a, ARM_JOINT_ROTATION, tau[4]);
    }
    RS00_Service(&rs00[0]);
    RS00_Service(&rs00[1]);
    RS05_Service(rs05);
    J4310_Service(j4310, tau[3], now);
    if (a == 1U)
      RS01_Service(&rs01_rarm);
    memcpy((void *)upper_state.board[a].gravity_tau_nm, tau, sizeof(tau));
    for (unsigned zero_joint = 0U; zero_joint < ARM_JOINT_COUNT; zero_joint++)
      Upper_ZeroCalibrationComplete(a, zero_joint);
    }
  }
}
