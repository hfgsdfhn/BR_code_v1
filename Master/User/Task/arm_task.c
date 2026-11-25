#include "arm_task.h"
#include "../Peripheral/CAN/bsp_can.h"
#include "cmsis_os2.h"
#include "cup_task.h"
#include "my_main.h"
#include "status_task.h"
#include "upper.h"
#include <math.h>
#include <string.h>

osMutexId_t br_mutex;
volatile float arm_target[2][ARM_JOINT_COUNT];
volatile float br_mit_tau[2][ARM_JOINT_COUNT];
volatile arm_mode_t arm_mode[2][ARM_JOINT_COUNT];
volatile uint8_t arm_remote_select;
volatile uint8_t arm_low_low_mode_enable;
volatile bool startup_hold_latched[2];
#define ARM_IK_X_MIN_CM (-100.0f)
#define ARM_IK_X_MAX_CM (100.0f)
#define ARM_IK_Y_MIN_CM (0.0f)
#define ARM_IK_Y_MAX_CM (100.0f)
#define ARM_IK_Z_MIN_CM (-100.0f)
#define ARM_IK_Z_MAX_CM (100.0f)

static int8_t arm_ik_target_direction[2][3] = {{-1, -1, -1}, {-1, -1, -1}};

static uint8_t Arm_SelectedRemoteArm(void) { return arm_remote_select != 0U ? ARM_RARM_INDEX : ARM_LARM_INDEX; }

static void Arm_SeedIKTargetFromFeedback(uint8_t arm)
{
    if (arm >= 2U)
        return;
    arm_end_target_world[arm][0] = arm_end_position_world[arm][0];
    arm_end_target_world[arm][1] = arm_end_position_world[arm][1] < ARM_IK_Y_MIN_CM ? ARM_IK_Y_MIN_CM : arm_end_position_world[arm][1];
    arm_end_target_world[arm][2] = arm_end_position_world[arm][2];
}

static void Arm_AdjustIKTarget(uint8_t arm, uint8_t axis, float delta_ms)
{
    if (arm >= 2U || axis >= 3U || delta_ms <= 0.0f)
        return;
    arm_end_target_world[arm][axis] += 0.01f * delta_ms * (float)arm_ik_target_direction[arm][axis];
    if (axis == 1U && arm_end_target_world[arm][axis] <= 10.0f)
        arm_end_target_world[arm][axis] = 10.1f;
}

volatile float arm_pose_pd11[2][ARM_JOINT_COUNT] = {{0.0f, 0.0f, 5.0f, 5.0f, 0.0f}, {0.0f, 0.0f, 5.0f, 5.0f, 0.0f}};
volatile float arm_pose_pd13[2][ARM_JOINT_COUNT] = {{0.0f, 80.0f, 80.0f, 0.0f, 0.0f}, {0.0f, 80.0f, 80.0f, 0.0f, 0.0f}};
volatile float arm_pose_pd13_1[2][ARM_JOINT_COUNT] = {{0.0f, 130.0f, 100.0f, -15.0f, 0.0f},{0.0f, 130.0f, 100.0f, -15.0f, 0.0f}};
volatile float arm_pose_pd13_2[2][ARM_JOINT_COUNT] = {{0.0f, 155.0f, 112.0f, -10.0f, 0.0f},{0.0f, 155.0f, 112.0f, -10.0f, 0.0f}};
volatile float arm_pose_pd14[2][ARM_JOINT_COUNT] = {{0.0f, 10.0f, 30.0f, 30.0f, 0.0f},{0.0f, 10.0f, 30.0f, 30.0f, 0.0f}};
volatile float arm_pose_pd15[2][ARM_JOINT_COUNT] = {{0.0f, 70.0f, 63.0f, 58.0f, 0.0f},{0.0f, 70.0f, 63.0f, 58.0f, 0.0f}};
volatile float arm_pose_pd15_2[2][ARM_JOINT_COUNT] = {{0.0f, 70.0f, 90.0f, 70.0f, 0.0f},{0.0f, 70.0f, 90.0f, 70.0f, 0.0f}};
volatile float xarm_pose_pd13[2][ARM_JOINT_COUNT] = {{-85.0f, 30.0f, 40.0f, -5.0f, 0.0f},{0.0f, 0.0f, 30.0f, 50.0f, 90.0f}};
volatile float xarm_pose_pd13_2[2][ARM_JOINT_COUNT] = {{-85.0f, 68.0f, 28.0f, -10.0f, 30.0f},{0.0f, 0.0f, 30.0f, 50.0f, 90.0f}};
volatile float xarm_pose_pd14[2][ARM_JOINT_COUNT] = {{-85.0f, 40.0f, 30.0f, 0.0f, 50.0f},{0.0f, 0.0f, 30.0f, 50.0f, 90.0f}};
volatile float xarm_pose_pd14_1[2][ARM_JOINT_COUNT] = {{0.0f, 0.0f, 30.0f, 30.0f, 0.0f},{0.0f, 0.0f, 30.0f, 50.0f, 90.0f}};
volatile float xarm_pose_pd14_2[2][ARM_JOINT_COUNT] = {{0.0f, 70.0f, 100.0f, 10.0f, 0.0f},{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
volatile float xarm_pose_pd15[2][ARM_JOINT_COUNT] = {{20.0f, 0.0f, 30.0f, 50.0f, 90.0f},{20.0f, 0.0f, 30.0f, 50.0f, 90.0f}};
volatile float xarm_pose_pd15_2[2][ARM_JOINT_COUNT] = {{0.0f, 0.0f, 30.0f, 50.0f, 90.0f},{0.0f, 0.0f, 30.0f, 50.0f, 90.0f}};
float br_joint_velocity[2][ARM_JOINT_COUNT];
bool br_grip[2];
volatile uint8_t arm_joint_zero_calibration[2][ARM_JOINT_COUNT];

void Arm_OnFrame(const can_frame_t *frame)
{
    Upper_OnFrame(&upper_state, frame);
    SuctionCup_OnFrame(frame);
    SCB_CleanDCache_by_Addr(arm_feedback_angle, sizeof(arm_feedback_angle));
    SCB_CleanDCache_by_Addr(suction_feedback_angle, sizeof(suction_feedback_angle));
}
void Arm_TaskInit(void)
{
    memset(&upper_state, 0, sizeof(upper_state));
    upper_state.enable = 1U;
    memset((void *)arm_target, 0, sizeof(arm_target));
    memset((void *)br_mit_tau, 0, sizeof(br_mit_tau));
    memset(br_joint_velocity, 0, sizeof(br_joint_velocity));
    memset(br_grip, 0, sizeof(br_grip));
    arm_remote_select = 0U;
    arm_low_low_mode_enable = 0U;
    memset((void *)arm_joint_zero_calibration, 0, sizeof(arm_joint_zero_calibration));
    memset((void *)startup_hold_latched, 0, sizeof(startup_hold_latched));
    memset((void *)arm_end_target_world, 0, sizeof(arm_end_target_world));
    memset((void *)arm_ik_enable, 0, sizeof(arm_ik_enable));
    memset((void *)arm_ik_second_rs00_prefer_small, 0, sizeof(arm_ik_second_rs00_prefer_small));
    for (unsigned arm = 0U; arm < 2U; ++arm)
        for (unsigned axis = 0U; axis < 3U; ++axis)
            arm_ik_target_direction[arm][axis] = -1;
    memset(&Arm_larm, 0, sizeof(Arm_larm));
    memset(&Arm_rarm, 0, sizeof(Arm_rarm));
    for (unsigned arm = 0; arm < 2U; arm++)
        for (unsigned joint = 0; joint < ARM_JOINT_COUNT; joint++)
            arm_mode[arm][joint] =
                (arm == ARM_LARM_INDEX && joint == ARM_JOINT_ROOT) ? ARM_MODE_POSITION : ARM_MODE_MIT;
}

void Arm_TaskStop(void) { upper_state.enable = 0U; }

bool Arm_TxQueueInit(void)
{
    if (br_mutex == NULL)
        br_mutex = osMutexNew(NULL);
    return br_mutex != NULL;
}

void Arm_TxTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        {
            uint32_t now = HAL_GetTick();
            Upper_Service(now);
        }
        osDelay(2U);
    }
}

static bool arm_set_command(uint8_t arm, arm_joint_t joint, arm_mode_t mode, float target, float tau)
{
    if (arm >= 2U || joint >= ARM_JOINT_COUNT || !isfinite(target) || !isfinite(tau) || br_mutex == NULL)
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    arm_mode[arm][joint] = mode;
    arm_target[arm][joint] = target;
    br_mit_tau[arm][joint] = tau;
    osMutexRelease(br_mutex);
    return true;
}

bool Upper_SetJointMIT(uint8_t arm, arm_joint_t joint, float tau_nm, float output_degrees)
{
    return arm_set_command(arm, joint, ARM_MODE_MIT, output_degrees, tau_nm);
}

bool Upper_SetJointModeTarget(uint8_t arm, arm_joint_t joint, arm_mode_t mode, float target_degrees)
{
    return arm_set_command(arm, joint, mode, target_degrees, 0.0f);
}

bool Upper_SetJointDegrees(uint8_t arm, arm_joint_t joint, float degrees)
{
    return arm_set_command(arm, joint, ARM_MODE_POSITION, degrees, 0.0f);
}

void Upper_QueueArmPositions(bool enable)
{
    upper_state.enable = enable ? 1U : 0U;
    suction_enable = enable ? 1U : 0U;
    if (!enable)
        Arm_TaskStop();
}

static void Arm_ApplyRemotePose(const volatile float pose[2][ARM_JOINT_COUNT], bool preserve_rotation)
{
    uint8_t arm = arm_remote_select != 0U ? ARM_RARM_INDEX : ARM_LARM_INDEX;
    if (pose == NULL || br_mutex == NULL)
        return;
    osMutexAcquire(br_mutex, osWaitForever);
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; joint++)
    {
        if (preserve_rotation && joint == ARM_JOINT_ROTATION)
            continue;
        arm_target[arm][joint] = pose[arm][joint];
    }
    osMutexRelease(br_mutex);
}

static void Arm_ApplyDualPose(const volatile float left_pose[ARM_JOINT_COUNT],
                              const volatile float right_pose[ARM_JOINT_COUNT])
{
    if (left_pose == NULL || right_pose == NULL || br_mutex == NULL)
        return;
    osMutexAcquire(br_mutex, osWaitForever);
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint)
    {
        arm_target[ARM_LARM_INDEX][joint] = left_pose[joint];
        arm_target[ARM_RARM_INDEX][joint] = right_pose[joint];
    }
    osMutexRelease(br_mutex);
}
static void Arm_ResetSoftwareZeros(void)
{
    unsigned i;
    M2006_ResetSoftwareZero(&m2006_larm);
    RS01_ResetSoftwareZero(&rs01_rarm);
    RS05_ResetSoftwareZero(&rs05_larm);
    RS05_ResetSoftwareZero(&rs05_rarm);
    for (i = 0U; i < 2U; ++i)
    {
        M2006_ResetSoftwareZero(&m2006_ucup[i]);
        M2006_ResetSoftwareZero(&m2006_dcup[i]);
    }
    M3508_ResetSoftwareZero(&m3508_dcup[0]);
}

static void Arm_RequestAllJointMechanicalZeros(void)
{
    unsigned arm;
    unsigned joint;
    for (arm = 0U; arm < 2U; ++arm)
        for (joint = ARM_JOINT_SHOULDER; joint < ARM_JOINT_ROTATION; ++joint)
            arm_joint_zero_calibration[arm][joint] = 1U;
}

static void Arm_ClearTargets(void)
{
    if (br_mutex != NULL)
    {
        osMutexAcquire(br_mutex, osWaitForever);
        memset((void *)arm_target, 0, sizeof(arm_target));
        osMutexRelease(br_mutex);
    }
    else
        memset((void *)arm_target, 0, sizeof(arm_target));
    memset((void *)suction_target, 0, sizeof(suction_target));
}
static void Arm_DisablePayloadCompensation(void)
{
    gpio_pd5 = 0U;
    gpio_pd6 = 0U;
}

static void Arm_DisableRemoteIK(void)
{
    Upper_SetArmIKEnabled(0U, false);
    Upper_SetArmIKEnabled(1U, false);
}
static void Arm_EnableRemoteIK(void)
{
    Upper_SetArmIKEnabled(0U, upper_state.enable != 0U);
    Upper_SetArmIKEnabled(1U, upper_state.enable != 0U);
}

static void Arm_ProcessRemote(const send_to_arm_t *status)
{
    static uint16_t previous_keys;
    static bool previous_keys_valid;
    static uint8_t pd14_11_index = 1U;
    static uint8_t pd14_10_index;
    static uint8_t pd13_11_index;
    static uint8_t pd15_11_index;
    static uint8_t pd15_10_index;
    static uint8_t pd13_10_index;
    static uint8_t pa11_rotation_index;
    static uint8_t previous_pa4;
    static bool previous_pa4_valid;
    static bool previous_pose_mode_valid;
    static bool previous_pa6_low_mode;
    static bool previous_pa5_low_mode;
    static bool previous_mode_valid;
    static uint32_t previous_ik_tick;
    static bool previous_ik_tick_valid;
    uint16_t current_keys, pressed_keys;
    uint8_t current_pa4;
    bool pa5_low_mode;
    if (status == NULL)
        return;

    /* Reset edge baselines while the remote is offline. */
    if (status->remote.online == 0U)
    {
        previous_pa4_valid = false;
        previous_keys_valid = false;
        previous_mode_valid = false;
        previous_pose_mode_valid = false;
        previous_ik_tick_valid = false;
        return;
    }

    current_pa4 = (uint8_t)((status->remote.switch_state >> 2U) & 1U);
    if (previous_pa4_valid && current_pa4 != previous_pa4)
    {
        Arm_ResetSoftwareZeros();
        Arm_RequestAllJointMechanicalZeros();
        Arm_ClearTargets();
    }
    previous_pa4 = current_pa4;
    previous_pa4_valid = true;
    current_keys = (uint16_t)status->remote.key[0] | ((uint16_t)(status->remote.key[1] & 1U) << 8);
    bool pa6_low_mode = ((status->remote.switch_state & (1U << 0U)) != 0U);
    if (!previous_keys_valid)
    {
        previous_keys = current_keys;
        previous_keys_valid = true;
        return;
    }
    pressed_keys = (uint16_t)(current_keys & (uint16_t)~previous_keys);
    pa5_low_mode = ((status->remote.switch_state & (1U << 1U)) != 0U);
    if (previous_mode_valid && (previous_pa6_low_mode != pa6_low_mode || previous_pa5_low_mode != pa5_low_mode))
    {
        arm_remote_select = 0U;
        pd13_11_index = 0U;
        pd14_11_index = 0U;
        pd15_11_index = 0U;
        pd13_10_index = 0U;
        pd14_10_index = 0U;
        pd15_10_index = 0U;
        pa11_rotation_index = 0U;
    }
    /* IK is available only while both PA6 and PA5 are low. */
    if (previous_mode_valid && previous_pa6_low_mode && previous_pa5_low_mode && !(pa6_low_mode && pa5_low_mode))
        Arm_DisableRemoteIK();
    if (arm_low_low_mode_enable != 0U && pa6_low_mode && pa5_low_mode && (!previous_mode_valid || !previous_pa6_low_mode || !previous_pa5_low_mode))
    {
        Arm_SeedIKTargetFromFeedback(Arm_SelectedRemoteArm());
        Arm_EnableRemoteIK();
        previous_ik_tick_valid = false;
    }
    previous_mode_valid = true;

    if (!pa6_low_mode && !pa5_low_mode &&
        (!previous_pose_mode_valid || previous_pa6_low_mode != pa6_low_mode || previous_pa5_low_mode != pa5_low_mode))
    {
        pd13_11_index = 0U;
        pd14_11_index = 0U;
        pd15_11_index = 0U;
    }
    previous_pa6_low_mode = pa6_low_mode;
    previous_pa5_low_mode = pa5_low_mode;
    previous_pose_mode_valid = true;

    if (pa6_low_mode && pa5_low_mode && arm_low_low_mode_enable == 0U)
    {
        Arm_DisableRemoteIK();
        if (pressed_keys & (1U << 1))
            arm_low_low_mode_enable ^= 1U;
        previous_ik_tick_valid = false;
        previous_keys = current_keys;
        return;
    }
    if (pa6_low_mode && pa5_low_mode)
    {
        uint8_t arm = Arm_SelectedRemoteArm();
        if (pressed_keys & (1U << 0))
        {
            arm_remote_select ^= 1U;
            arm = Arm_SelectedRemoteArm();
            Arm_SeedIKTargetFromFeedback(arm);
        }
        if (pressed_keys & (1U << 1))
        {
            arm_ik_second_rs00_prefer_small[arm] ^= 1U;
            if (arm_ik_enable[arm] != 0U)
                Upper_RefreshArmIK(arm);
        }
        /* PD12 controls the total arm enable and synchronizes IK in this mode only. */
        if (pressed_keys & (1U << 2))
        {
            upper_state.enable ^= 1U;
            Arm_EnableRemoteIK();
        }
        uint32_t now_ik_tick = HAL_GetTick();
        float ik_delta_ms = previous_ik_tick_valid ? (float)(now_ik_tick - previous_ik_tick) : 0.0f;
        previous_ik_tick = now_ik_tick;
        previous_ik_tick_valid = true;
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            uint16_t mask = (uint16_t)(1U << (3U + axis));
            if (pressed_keys & mask)
            {
                arm_ik_target_direction[arm][axis] = (int8_t)-arm_ik_target_direction[arm][axis];
                if (axis == 0U)
                {
                    pd14_11_index = 0U;
                    pd15_11_index = 0U;
                }
                else if (axis == 1U)
                {
                    pd13_11_index = 0U;
                    pd15_11_index = 0U;
                }
                else
                {
                    pd13_11_index = 0U;
                    pd14_11_index = 0U;
                }
            }
            if (current_keys & mask)
            {
                Arm_AdjustIKTarget(arm, axis, ik_delta_ms);
                Upper_RefreshArmIK(arm);
            }
        }
        if (pressed_keys & (1U << 6))
            gpio_pd5 ^= 1U;
        if (pressed_keys & (1U << 7))
        {
            arm_target[arm][ARM_JOINT_ROTATION] =
                pa11_rotation_index == 0U ? 90.0f : (pa11_rotation_index == 1U ? 180.0f : 0.0f);
            pa11_rotation_index = (uint8_t)((pa11_rotation_index + 1U) % 3U);
        }
        if (pressed_keys & (1U << 8))
            gpio_pd6 ^= 1U;
        previous_keys = current_keys;
        return;
    }

    if (!pa6_low_mode && (pressed_keys & (1U << 0)) != 0U)
    {
        arm_remote_select ^= 1U;
        pd13_11_index = 0U;
        pd14_11_index = 0U;
        pd15_11_index = 0U;
    }
    if (!pa6_low_mode && (pressed_keys & (1U << 2)) != 0U)
        upper_state.enable ^= 1U;
    if (!pa6_low_mode && !pa5_low_mode)
    {
        if ((pressed_keys & (1U << 3)) != 0U)
        {
            if (pd13_11_index == 0U)
                Arm_ApplyRemotePose(arm_pose_pd13, true);
            else if (pd13_11_index == 1U)
                Arm_ApplyRemotePose(arm_pose_pd13_1, true);
            else
                Arm_ApplyRemotePose(arm_pose_pd13_2, true);
            pd13_11_index = (uint8_t)((pd13_11_index + 1U) % 3U);
            pd14_11_index = 0U;
            pd15_11_index = 0U;
        }
        if ((pressed_keys & (1U << 4)) != 0U)
        {
            Arm_ApplyRemotePose(arm_pose_pd14, true);
            pd14_11_index = (uint8_t)((pd14_11_index + 1U) % 2U);
            pd13_11_index = 0U;
            pd15_11_index = 0U;
        }
        if ((pressed_keys & (1U << 5)) != 0U)
        {
            if (pd15_11_index == 0U)
            {
                Arm_ApplyRemotePose(arm_pose_pd15, true);
            }
            else
            {
                Arm_ApplyRemotePose(arm_pose_pd15_2, true);
            }
            pd15_11_index = (uint8_t)((pd15_11_index + 1U) % 2U);
            pd13_11_index = 0U;
            pd14_11_index = 0U;
        }
        if ((pressed_keys & (1U << 1)) != 0U)
        {
            Arm_ApplyRemotePose(arm_pose_pd11, false);
            pd13_11_index = 0U;
            pd14_11_index = 0U;
            pd15_11_index = 0U;
        }
    }
    else if (!pa6_low_mode)
    {
        if ((pressed_keys & (1U << 3)) != 0U)
        {
            const volatile float (*pose)[ARM_JOINT_COUNT] = pd13_10_index == 0U ? xarm_pose_pd13 : xarm_pose_pd13_2;
            Arm_ApplyDualPose(pose[ARM_LARM_INDEX], pose[ARM_RARM_INDEX]);
            pd13_10_index = (uint8_t)((pd13_10_index + 1U) % 2U);
            pd14_10_index = 0U;
            pd15_10_index = 0U;
        }
        if ((pressed_keys & (1U << 4)) != 0U)
        {
            const volatile float (*pose)[ARM_JOINT_COUNT] =
                pd14_10_index == 0U ? xarm_pose_pd14 : (pd14_10_index == 1U ? xarm_pose_pd14_1 : xarm_pose_pd14_2);
            Arm_ApplyDualPose(pose[ARM_LARM_INDEX], pose[ARM_RARM_INDEX]);
            pd14_10_index = (uint8_t)((pd14_10_index + 1U) % 3U);
            pd13_10_index = 0U;
            pd15_10_index = 0U;
        }
        if ((pressed_keys & (1U << 5)) != 0U)
        {
            const volatile float (*pose)[ARM_JOINT_COUNT] = pd15_10_index == 0U ? xarm_pose_pd15 : xarm_pose_pd15_2;
            Arm_ApplyDualPose(pose[ARM_LARM_INDEX], pose[ARM_RARM_INDEX]);
            pd15_10_index = (uint8_t)((pd15_10_index + 1U) % 2U);
            pd13_10_index = 0U;
            pd14_10_index = 0U;
        }
        if ((pressed_keys & (1U << 1)) != 0U)
        {
            Arm_ApplyDualPose(arm_pose_pd11[ARM_LARM_INDEX], arm_pose_pd11[ARM_RARM_INDEX]);
            pd13_10_index = 0U;
            pd14_10_index = 0U;
            pd15_10_index = 0U;
        }
    }
    if (!pa6_low_mode)
    {
        if ((pressed_keys & (1U << 6)) != 0U)
            gpio_pd5 ^= 1U;
        if ((pressed_keys & (1U << 7)) != 0U)
        {
            uint8_t arm = Arm_SelectedRemoteArm();
            arm_target[arm][ARM_JOINT_ROTATION] =
                pa11_rotation_index == 0U ? 90.0f : (pa11_rotation_index == 1U ? 180.0f : 0.0f);
            pa11_rotation_index = (uint8_t)((pa11_rotation_index + 1U) % 3U);
        }
        if ((pressed_keys & (1U << 8)) != 0U)
            gpio_pd6 ^= 1U;
    }
    previous_keys = current_keys;
}
void Arm_Task(void *argument)
{
    uint32_t next = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        send_to_arm_t status;
        if (osMessageQueueGet(Arm_QueueHandle, &status, NULL, 0U) == osOK)
            Arm_ProcessRemote(&status);
        next += (osKernelGetTickFreq() * 2U + 999U) / 1000U;
        if (osDelayUntil(next) != osOK)
            next = osKernelGetTickCount();
    }
}
