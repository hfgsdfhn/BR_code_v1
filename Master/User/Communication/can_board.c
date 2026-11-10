#include "cmsis_os2.h"
#include "my_main.h"
#include "upper.h"
#include <math.h>
#include <string.h>
upper_t upper_state;
Arm_Pos Arm_larm, Arm_rarm;
/* All driver state is owned by the CAN service and protected against the RX task. */
osMutexId_t br_mutex;
float br_target[2][4];
float br_root_target[2];
float br_mit_tau[2][ARM_JOINT_COUNT]; /* Motor output-shaft Nm, signed in motor coordinates. */
rs_mode_t br_joint_mode[2][ARM_JOINT_COUNT];
float br_joint_velocity[2][ARM_JOINT_COUNT];
uint32_t br_command_tick[2];
bool br_command_enable[2], br_grip[2];
static bool position_valid(unsigned a, const Arm_Pos *p)
{
    const float lo[4] = BR_JOINT_MIN_RAD, hi[4] = BR_JOINT_MAX_RAD;
    const float direction[2][4] = {BR_LEFT_DIRECTION, BR_RIGHT_DIRECTION};
    const float zero[2][4] = {BR_LEFT_ZERO_RAD, BR_RIGHT_ZERO_RAD};
    const float ratio[2][4] = {BR_LEFT_JOINT_PER_MOTOR, BR_RIGHT_JOINT_PER_MOTOR};
    float q[4];
    unsigned i;
    if (!p || !isfinite(p->Root) || p->Root * ARM_DEG_TO_RAD < BR_ROOT_MIN_RAD ||
        p->Root * ARM_DEG_TO_RAD > BR_ROOT_MAX_RAD)
        return false;
    q[0] = p->RS00_1 * ARM_DEG_TO_RAD;
    q[1] = p->RS00_2 * ARM_DEG_TO_RAD;
    q[2] = p->J4310 * ARM_DEG_TO_RAD;
    q[3] = p->RS05 * ARM_DEG_TO_RAD;
    for (i = 0; i < 4; i++)
    {
        float motor = zero[a][i] + direction[a][i] * q[i] / ratio[a][i];
        if (!isfinite(q[i]) || q[i] < lo[i] || q[i] > hi[i] || !isfinite(motor) ||
            fabsf(motor) > (i == 2 ? BR_DM_POSITION_MAX : BR_RS_POSITION_MAX))
            return false;
    }
    if (a == 1 && fabsf(BR_RIGHT_ROOT_ZERO_RAD + BR_ROOT_RS02_DIRECTION * p->Root * ARM_DEG_TO_RAD /
                                                     BR_RIGHT_ROOT_JOINT_PER_MOTOR) > BR_RS_POSITION_MAX)
        return false;
    return true;
}
static void queue_locked(unsigned a, const Arm_Pos *p, bool enable)
{
    if (!position_valid(a, p))
    {
        br_command_enable[a] = false;
        return;
    }
    br_target[a][0] = p->RS00_1 * ARM_DEG_TO_RAD;
    br_target[a][1] = p->RS00_2 * ARM_DEG_TO_RAD;
    br_target[a][2] = p->J4310 * ARM_DEG_TO_RAD;
    br_target[a][3] = p->RS05 * ARM_DEG_TO_RAD;
    br_root_target[a] = p->Root * ARM_DEG_TO_RAD;
    br_command_tick[a] = HAL_GetTick();
    br_command_enable[a] = enable;
}
void Upper_Init(upper_t *u)
{
    if (u)
        memset(u, 0, sizeof(*u));
}
void Arm_Init(void)
{
    memset(&Arm_larm, 0, sizeof(Arm_larm));
    memset(&Arm_rarm, 0, sizeof(Arm_rarm));
    memset(br_command_enable, 0, sizeof(br_command_enable));
    {
        unsigned a, j;
        for (a = 0; a < 2; a++)
            for (j = 0; j < ARM_JOINT_COUNT; j++)
                br_joint_mode[a][j] = RS_MODE_POSITION;
    }
    memset(br_mit_tau, 0, sizeof(br_mit_tau));
    memset(br_target, 0, sizeof(br_target));
    memset(br_root_target, 0, sizeof(br_root_target));
    memset(br_grip, 0, sizeof(br_grip));
    memset(br_joint_velocity, 0, sizeof(br_joint_velocity));
    Upper_DirectInit();
}
bool Upper_TxQueueInit(void)
{
    if (!br_mutex)
        br_mutex = osMutexNew(NULL);
    return br_mutex != NULL;
}
bool Upper_SetArmDegrees(uint8_t a, const Arm_Pos *p)
{
    if (a >= 2 || !position_valid(a, p) || !br_mutex)
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    if (a == 0)
        Arm_larm = *p;
    else
        Arm_rarm = *p;
    {
        unsigned j;
        for (j = 0; j < ARM_JOINT_COUNT; j++)
        {
            br_joint_mode[a][j] = RS_MODE_POSITION;
            br_joint_velocity[a][j] = 0;
            br_mit_tau[a][j] = 0;
        }
    }
    queue_locked(a, p, upper_state.enable != 0);
    osMutexRelease(br_mutex);
    return true;
}
void Upper_QueueArmPositions(bool enable)
{
    if (!br_mutex)
        return;
    osMutexAcquire(br_mutex, osWaitForever);
    queue_locked(0, &Arm_larm, enable);
    queue_locked(1, &Arm_rarm, enable);
    osMutexRelease(br_mutex);
}
void Upper_SetGripSignal(uint8_t a, bool detected, uint8_t source)
{
    (void)source;
    if (a >= 2 || !br_mutex)
        return;
    osMutexAcquire(br_mutex, osWaitForever);
    br_grip[a] = detected;
    osMutexRelease(br_mutex);
}
void Upper_Stop(upper_t *u)
{
    if (u)
        u->enable = 0;
    g_motor_control_enable = 0U;
    Upper_QueueArmPositions(false);
}
void Upper_TxTask(void *argument)
{
    uint32_t next = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        osMutexAcquire(br_mutex, osWaitForever);
        Upper_DirectService(HAL_GetTick());
        osMutexRelease(br_mutex);
        next += (osKernelGetTickFreq() * BR_PERIOD_MS + 999U) / 1000U;
        if (osDelayUntil(next) != osOK)
            next = osKernelGetTickCount();
    }
}
void Upper_OnFrame(upper_t *u, const can_frame_t *f)
{
    if (!u || !f || !br_mutex)
        return;
    osMutexAcquire(br_mutex, osWaitForever);
    Upper_DirectFeedback(u, f);
    osMutexRelease(br_mutex);
}
void Upper_SendBoardTargets(upper_t *u, uint8_t a, const float q[ARM_JOINT_COUNT])
{
    Arm_Pos p;
    if (!u || !q || a >= 2)
        return;
    p.Root = q[4] * ARM_RAD_TO_DEG;
    p.RS05 = q[0] * ARM_RAD_TO_DEG;
    p.J4310 = q[1] * ARM_RAD_TO_DEG;
    p.RS00_2 = q[2] * ARM_RAD_TO_DEG;
    p.RS00_1 = q[3] * ARM_RAD_TO_DEG;
    (void)Upper_SetArmDegrees(a, &p);
}
void Upper_SendTargets(upper_t *u, const float q[2][ARM_JOINT_COUNT])
{
    if (q)
    {
        Upper_SendBoardTargets(u, 0, q[0]);
        Upper_SendBoardTargets(u, 1, q[1]);
    }
}
void Upper_GetJointRad(const upper_t *u, uint8_t a, float out[ARM_JOINT_COUNT])
{
    unsigned i;
    if (!u || !out || a >= 2 || !br_mutex)
        return;
    osMutexAcquire(br_mutex, osWaitForever);
    for (i = 0; i < ARM_JOINT_COUNT; i++)
        out[i] = u->board[a].joint_mrad[i] * 0.001f;
    osMutexRelease(br_mutex);
}

bool Upper_SetJointDegrees(uint8_t a, arm_joint_t joint, float degrees)
{
    Arm_Pos p;
    if (a >= 2 || joint < ARM_JOINT_ROTATION || joint > ARM_JOINT_ROOT || !isfinite(degrees) || !br_mutex)
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    p = a ? Arm_rarm : Arm_larm;
    switch (joint)
    {
    case ARM_JOINT_ROTATION:
        p.RS05 = degrees;
        break;
    case ARM_JOINT_WRIST:
        p.J4310 = degrees;
        break;
    case ARM_JOINT_ELBOW:
        p.RS00_2 = degrees;
        break;
    case ARM_JOINT_SHOULDER:
        p.RS00_1 = degrees;
        break;
    case ARM_JOINT_ROOT:
        p.Root = degrees;
        break;
    default:
        osMutexRelease(br_mutex);
        return false;
    }
    if (!position_valid(a, &p))
    {
        osMutexRelease(br_mutex);
        return false;
    }
    if (a)
        Arm_rarm = p;
    else
        Arm_larm = p;
    br_joint_mode[a][joint] = RS_MODE_POSITION;
    br_joint_velocity[a][joint] = 0;
    br_mit_tau[a][joint] = 0;
    queue_locked(a, &p, upper_state.enable != 0);
    osMutexRelease(br_mutex);
    return true;
}
bool Upper_SetM2006PositionDegrees(uint8_t bus, uint8_t index, float degrees)
{
    return Upper_SetM2006(bus, index, M2006_MODE_POSITION, degrees);
}
bool Upper_SetM3508PositionDegrees(float degrees)
{
    return Upper_SetM3508(M3508_MODE_POSITION, degrees);
}

bool Upper_SetJointModeTarget(uint8_t a, arm_joint_t joint, rs_mode_t mode, float target)
{
    if (a >= 2 || joint < ARM_JOINT_ROTATION || joint > ARM_JOINT_ROOT || !isfinite(target) || !br_mutex)
        return false;
    if (mode == RS_MODE_POSITION)
        return Upper_SetJointDegrees(a, joint, target);
    if (mode != RS_MODE_STOP && mode != RS_MODE_DAMPING && mode != RS_MODE_SPEED)
        return false;
    if (mode == RS_MODE_SPEED && fabsf(target) > BR_POSITION_SLEW_RAD_S)
        return false;
    if (mode == RS_MODE_DAMPING && target != 0.0f)
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    br_mit_tau[a][joint] = 0;
    br_joint_mode[a][joint] = mode;
    br_joint_velocity[a][joint] = mode == RS_MODE_SPEED ? target : 0.0f;
    br_command_tick[a] = HAL_GetTick();
    br_command_enable[a] = upper_state.enable != 0;
    osMutexRelease(br_mutex);
    return true;
}

bool Upper_SetJointMIT(uint8_t a, arm_joint_t joint, float tau_nm, float output_degrees)
{
    const float dirs[2][4] = {BR_LEFT_DIRECTION, BR_RIGHT_DIRECTION};
    const float zeros[2][4] = {BR_LEFT_ZERO_RAD, BR_RIGHT_ZERO_RAD};
    const float ratios[2][4] = {BR_LEFT_JOINT_PER_MOTOR, BR_RIGHT_JOINT_PER_MOTOR};
    float q, tmax, pmax, p = output_degrees * ARM_DEG_TO_RAD;
    Arm_Pos pos;
    if (a >= 2 || joint < ARM_JOINT_ROTATION || joint > ARM_JOINT_ROOT || !br_mutex || !isfinite(p) ||
        !isfinite(tau_nm) || (a == 0 && joint == ARM_JOINT_ROOT))
        return false;
    if (joint == ARM_JOINT_ROOT)
    {
        q = BR_ROOT_RS02_DIRECTION * (p - BR_RIGHT_ROOT_ZERO_RAD) * BR_RIGHT_ROOT_JOINT_PER_MOTOR;
        tmax = BR_RS02_TORQUE_MAX;
        pmax = BR_RS_POSITION_MAX;
    }
    else
    {
        unsigned index = 3 - (unsigned)joint;
        q = dirs[a][index] * (p - zeros[a][index]) * ratios[a][index];
        tmax = joint == ARM_JOINT_WRIST ? BR_DM_TORQUE_MAX
                                        : (joint == ARM_JOINT_ROTATION ? BR_RS05_TORQUE_MAX : BR_RS00_TORQUE_MAX);
        pmax = joint == ARM_JOINT_WRIST ? BR_DM_POSITION_MAX : BR_RS_POSITION_MAX;
    }
    if (fabsf(p) > pmax || fabsf(tau_nm) > tmax)
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    pos = a ? Arm_rarm : Arm_larm;
    switch (joint)
    {
    case ARM_JOINT_ROTATION:
        pos.RS05 = q * ARM_RAD_TO_DEG;
        break;
    case ARM_JOINT_WRIST:
        pos.J4310 = q * ARM_RAD_TO_DEG;
        break;
    case ARM_JOINT_ELBOW:
        pos.RS00_2 = q * ARM_RAD_TO_DEG;
        break;
    case ARM_JOINT_SHOULDER:
        pos.RS00_1 = q * ARM_RAD_TO_DEG;
        break;
    case ARM_JOINT_ROOT:
        pos.Root = q * ARM_RAD_TO_DEG;
        break;
    default:
        osMutexRelease(br_mutex);
        return false;
    }
    if (!position_valid(a, &pos))
    {
        osMutexRelease(br_mutex);
        return false;
    }
    if (a)
        Arm_rarm = pos;
    else
        Arm_larm = pos;
    br_joint_mode[a][joint] = RS_MODE_MIT;
    br_joint_velocity[a][joint] = 0;
    br_mit_tau[a][joint] = tau_nm;
    queue_locked(a, &pos, upper_state.enable != 0);
    osMutexRelease(br_mutex);
    return true;
}
