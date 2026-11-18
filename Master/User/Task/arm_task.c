
#include "arm_task.h"
#include "cmsis_os2.h"
#include "my_main.h"
#include "upper.h"
#include "cup_task.h"
#include "status_task.h"
#include "../Peripheral/CAN/bsp_can.h"
#include <math.h>
#include <string.h>

osMutexId_t br_mutex;
volatile float arm_target[2][ARM_JOINT_COUNT];
volatile float br_mit_tau[2][ARM_JOINT_COUNT];
volatile arm_mode_t arm_mode[2][ARM_JOINT_COUNT];
volatile uint8_t arm_remote_select;
static int8_t arm_ik_target_direction[2][3] = {
    {-1, -1, -1},
    {-1, -1, -1}
};

static uint8_t Arm_SelectedRemoteArm(void)
{
    return arm_remote_select != 0U ? ARM_RARM_INDEX : ARM_LARM_INDEX;
}

static void Arm_SeedIKTargetFromFeedback(uint8_t arm)
{
    if (arm >= 2U)
        return;
    arm_end_target_world[arm][0] = arm_end_position_world[arm][0];
    arm_end_target_world[arm][1] = arm_end_position_world[arm][1] <= 10.0f
                                        ? 10.1f : arm_end_position_world[arm][1];
    arm_end_target_world[arm][2] = arm_end_position_world[arm][2];
}

static void Arm_AdjustIKTarget(uint8_t arm, uint8_t axis)
{
    if (arm >= 2U || axis >= 3U)
        return;
    arm_end_target_world[arm][axis] +=
        0.1f * (float)arm_ik_target_direction[arm][axis];
    if (axis == 1U && arm_end_target_world[arm][axis] <= 10.0f)
        arm_end_target_world[arm][axis] = 10.1f;
}

volatile float arm_pose_pd11[2][ARM_JOINT_COUNT] = {{0.0f, 20.0f, 20.0f, 20.0f, 0.0f}, {0.0f, 20.0f, 20.0f, 20.0f, 0.0f}};
volatile float arm_pose_pd13[2][ARM_JOINT_COUNT] = {{0.0f, 230.0f, 90.0f, 0.0f, 0.0f}, {0.0f, 230.0f, 90.0f, 0.0f, 0.0f}};
volatile float arm_pose_pd14[2][ARM_JOINT_COUNT] = {{0.0f, 20.0f, 40.0f, 10.0f, 0.0f}, {0.0f, 20.0f, 40.0f, 10.0f, 0.0f}};
volatile float arm_pose_pd14_2[2][ARM_JOINT_COUNT] = {{0.0f, 120.0f, 85.0f, 40.0f, 0.0f}, {0.0f, 120.0f, 85.0f, 40.0f, 0.0f}};
volatile float arm_pose_pd15[2][ARM_JOINT_COUNT] = {{0.0f, 110.0f, 100.0f, 20.0f, 0.0f}, {0.0f, 110.0f, 100.0f, 20.0f, 0.0f}};
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
    memset((void *)arm_target, 0, sizeof(arm_target));
    memset((void *)br_mit_tau, 0, sizeof(br_mit_tau));
    memset(br_joint_velocity, 0, sizeof(br_joint_velocity));
    memset(br_grip, 0, sizeof(br_grip));
    arm_remote_select = 0U;
    memset((void *)arm_joint_zero_calibration, 0, sizeof(arm_joint_zero_calibration));
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
            arm_mode[arm][joint] = (arm == ARM_LARM_INDEX && joint == ARM_JOINT_ROOT)
                                        ? ARM_MODE_POSITION
                                        : ARM_MODE_MIT;
}

void Arm_TaskStop(void)
{
    upper_state.enable = 0U;
}

bool Arm_TxQueueInit(void)
{
    if (br_mutex == NULL)
        br_mutex = osMutexNew(NULL);
    return br_mutex != NULL;
}

void Arm_TxTask(void *argument)
{
    (void)argument;
    for (;;) {
        { uint32_t now = HAL_GetTick(); Upper_Service(now); }
        osDelay(1U);
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

static void Arm_ResetAllMotorSoftwareZeros(void)
{
    unsigned i;
    M2006_ResetSoftwareZero(&m2006_larm);
    for (i = 0U; i < 2U; i++) {
        M2006_ResetSoftwareZero(&m2006_ucup[i]);
        M2006_ResetSoftwareZero(&m2006_dcup[i]);
    }
    M3508_ResetSoftwareZero(&m3508_dcup[0]);
    RS01_ResetSoftwareZero(&rs01_rarm);
}


static void Arm_RequestAllJointMechanicalZeros(void)
{
    unsigned arm;
    unsigned joint;
    for (arm = 0U; arm < 2U; ++arm)
        for (joint = 0U; joint < ARM_JOINT_COUNT; ++joint)
            arm_joint_zero_calibration[arm][joint] = 1U;
}

static void Arm_ProcessRemote(const send_to_arm_t *status)
{
    static uint16_t previous_keys;
    static bool previous_keys_valid;
    static uint8_t pd14_pose_index = 1U;
    static uint8_t previous_pa4;
    static bool previous_pa4_valid;
    uint16_t current_keys;
    uint16_t pressed_keys;
    uint8_t current_pa4;
    bool pa5_low_mode;
    if (status == NULL)
        return;
    current_pa4 = (uint8_t)((status->remote.switch_state >> 2U) & 1U);
    if (previous_pa4_valid && current_pa4 != previous_pa4)
    {
        Arm_ResetAllMotorSoftwareZeros();
        Arm_RequestAllJointMechanicalZeros();
    }
    previous_pa4 = current_pa4;
    previous_pa4_valid = true;

    current_keys = (uint16_t)status->remote.key[0] |
                   ((uint16_t)(status->remote.key[1] & 1U) << 8);
    if ((status->remote.switch_state & (1U << 0)) != 0U)
    {
        previous_keys = 0U;
        previous_keys_valid = false;
        return;
    }
    if (!previous_keys_valid)
    {
        previous_keys = current_keys;
        previous_keys_valid = true;
        return;
    }
    pressed_keys = (uint16_t)(current_keys & (uint16_t)~previous_keys);
    pa5_low_mode = ((status->remote.switch_state & (1U << 1U)) != 0U);
    if ((pressed_keys & (1U << 0)) != 0U) arm_remote_select ^= 1U;
    if (!pa5_low_mode && (pressed_keys & (1U << 2)) != 0U) upper_state.enable ^= 1U;
    if (pa5_low_mode)
    {
        if ((pressed_keys & (1U << 2)) != 0U)
        {
            uint8_t arm = Arm_SelectedRemoteArm();
            uint8_t enable = arm_ik_enable[arm] == 0U ? 1U : 0U;
            arm_ik_enable[arm] = enable;
            upper_state.enable = enable;
            if (enable != 0U)
                Arm_SeedIKTargetFromFeedback(arm);
            Upper_RefreshArmIK(arm);
        }
        if ((pressed_keys & (1U << 3)) != 0U)
            arm_ik_target_direction[Arm_SelectedRemoteArm()][0] *= -1;
        if ((pressed_keys & (1U << 4)) != 0U)
            arm_ik_target_direction[Arm_SelectedRemoteArm()][1] *= -1;
        if ((pressed_keys & (1U << 5)) != 0U)
            arm_ik_target_direction[Arm_SelectedRemoteArm()][2] *= -1;
        if ((current_keys & (1U << 3)) != 0U)
            Arm_AdjustIKTarget(Arm_SelectedRemoteArm(), 0U);
        if ((current_keys & (1U << 4)) != 0U)
            Arm_AdjustIKTarget(Arm_SelectedRemoteArm(), 1U);
        if ((current_keys & (1U << 5)) != 0U)
            Arm_AdjustIKTarget(Arm_SelectedRemoteArm(), 2U);
        if ((pressed_keys & (1U << 1)) != 0U)
        {
            uint8_t arm = arm_remote_select != 0U ? ARM_RARM_INDEX : ARM_LARM_INDEX;
            arm_ik_second_rs00_prefer_small[arm] ^= 1U;
            Upper_RefreshArmIK(arm);
        }
        if ((pressed_keys & (1U << 6)) != 0U) gpio_pd5 ^= 1U;
        if ((pressed_keys & (1U << 8)) != 0U) gpio_pd6 ^= 1U;
        if ((pressed_keys & (1U << 7)) != 0U)
        {
            uint8_t arm = arm_remote_select != 0U ? ARM_RARM_INDEX : ARM_LARM_INDEX;
            arm_target[arm][ARM_JOINT_ROTATION] =
                (arm_target[arm][ARM_JOINT_ROTATION] == 180.0f) ? 0.0f : 180.0f;
        }
        previous_keys = current_keys;
        return;
    }
    if ((pressed_keys & (1U << 3)) != 0U)
    {
        pd14_pose_index = 1U;
        Arm_ApplyRemotePose(arm_pose_pd13, true);
    }
    if ((pressed_keys & (1U << 4)) != 0U)
    {
        pd14_pose_index = (uint8_t)((pd14_pose_index + 1U) % 2U);
        if (pd14_pose_index == 0U) Arm_ApplyRemotePose(arm_pose_pd14, true);
        else Arm_ApplyRemotePose(arm_pose_pd14_2, true);
    }
    if ((pressed_keys & (1U << 5)) != 0U)
    {
        pd14_pose_index = 1U;
        Arm_ApplyRemotePose(arm_pose_pd15, true);
    }
    if ((pressed_keys & (1U << 1)) != 0U)
    {
        pd14_pose_index = 1U;
        Arm_ApplyRemotePose(arm_pose_pd11, false);
    }
    if ((pressed_keys & (1U << 7)) != 0U)
    {
        uint8_t arm = arm_remote_select != 0U ? ARM_RARM_INDEX : ARM_LARM_INDEX;
        arm_target[arm][ARM_JOINT_ROTATION] =
            (arm_target[arm][ARM_JOINT_ROTATION] == 180.0f) ? 0.0f : 180.0f;
    }
    if ((pressed_keys & (1U << 6)) != 0U) gpio_pd5 ^= 1U;
    if ((pressed_keys & (1U << 8)) != 0U) gpio_pd6 ^= 1U;
    previous_keys = current_keys;
}

void Arm_Task(void *argument)
{
    uint32_t next = osKernelGetTickCount();
    (void)argument;
    for (;;) {
        send_to_arm_t status;
        if (osMessageQueueGet(Arm_QueueHandle, &status, NULL, 0U) == osOK)
            Arm_ProcessRemote(&status);
        next += (osKernelGetTickFreq() * 2U + 999U) / 1000U;
        if (osDelayUntil(next) != osOK)
            next = osKernelGetTickCount();
    }
}
