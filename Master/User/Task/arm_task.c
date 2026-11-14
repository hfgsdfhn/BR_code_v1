/*
 * @file arm_task.c
 * @brief arm_task模块的用户接口与实现。
 */

/**
 * @file arm_task.c
 * @brief Arm task state and target management.
 */

#include "arm_task.h"
#include "cmsis_os2.h"
#include "my_main.h"
#include "upper.h"
#include "suction_cup.h"
#include "../Peripheral/CAN/bsp_can.h"
#include <math.h>
#include <string.h>

osMutexId_t br_mutex;
volatile float arm_target[2][ARM_JOINT_COUNT];
volatile float br_mit_tau[2][ARM_JOINT_COUNT];
volatile arm_mode_t arm_mode[2][ARM_JOINT_COUNT];
float br_joint_velocity[2][ARM_JOINT_COUNT];
bool br_grip[2];

/* 代码说明：相关模块实现。 */
volatile uint8_t arm_gpio_pb6 = 0U;
volatile uint8_t arm_gpio_pb7 = 0U;
volatile uint8_t arm_gpio_pd7 = 0U;
volatile uint8_t arm_gpio_pd6 = 0U;
volatile uint8_t arm_gpio_pd1 = 0U;
volatile uint8_t arm_gpio_pd3 = 0U;
volatile uint8_t arm_gpio_pd4 = 0U;
volatile uint8_t arm_gpio_pd5 = 0U;
volatile uint8_t arm_joint_zero_calibration[2][ARM_JOINT_COUNT];


/* 功能：执行 Arm_OnFrame 函数对应的模块操作。 */
void Arm_OnFrame(const can_frame_t *frame)
{
    Upper_OnFrame(&upper_state, frame);
    SuctionCup_OnFrame(frame);
    /* 代码说明：相关模块实现。 */
    /* volatile alone does not write dirty cache lines back to AXI SRAM. */
    SCB_CleanDCache_by_Addr(arm_feedback_angle, sizeof(arm_feedback_angle));
    SCB_CleanDCache_by_Addr(suction_feedback_angle, sizeof(suction_feedback_angle));
}
/* 功能：执行 Arm_ServiceGpio 函数对应的模块操作。 */
static void Arm_ServiceGpio(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,
                      arm_gpio_pb6 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,
                      arm_gpio_pb7 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,
                      arm_gpio_pd7 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,
                      arm_gpio_pd6 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,
                      arm_gpio_pd1 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,
                      arm_gpio_pd3 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,
                      arm_gpio_pd4 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,
                      arm_gpio_pd5 != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 功能：执行 Arm_TaskInit 函数对应的模块操作。 */
void Arm_TaskInit(void)
{
    memset(&upper_state, 0, sizeof(upper_state));
    memset((void *)arm_target, 0, sizeof(arm_target));
    memset((void *)br_mit_tau, 0, sizeof(br_mit_tau));
    memset(br_joint_velocity, 0, sizeof(br_joint_velocity));
    memset(br_grip, 0, sizeof(br_grip));
    arm_gpio_pb6 = 0U;
    arm_gpio_pb7 = 0U;
    arm_gpio_pd7 = 0U;
    arm_gpio_pd6 = 0U;
    arm_gpio_pd1 = 0U;
    arm_gpio_pd3 = 0U;
    arm_gpio_pd4 = 0U;
    arm_gpio_pd5 = 0U;
    memset((void *)arm_joint_zero_calibration, 0, sizeof(arm_joint_zero_calibration));
    memset(&Arm_larm, 0, sizeof(Arm_larm));
    memset(&Arm_rarm, 0, sizeof(Arm_rarm));
    /* Prepare 位置 commands; upper_state.使能 gates 电机 输出. */
    for (unsigned arm = 0; arm < 2U; arm++)
        for (unsigned joint = 0; joint < ARM_JOINT_COUNT; joint++)
            arm_mode[arm][joint] = ARM_MODE_POSITION;
}

/* 功能：执行 Arm_TaskStop 函数对应的模块操作。 */
void Arm_TaskStop(void)
{
    upper_state.enable = 0U;
}

/* 功能：执行 Arm_TxQueueInit 函数对应的模块操作。 */
bool Arm_TxQueueInit(void)
{
    if (br_mutex == NULL)
        br_mutex = osMutexNew(NULL);
    return br_mutex != NULL;
}

/* 功能：执行 Arm_TxTask 函数对应的模块操作。 */
void Arm_TxTask(void *argument)
{
    (void)argument;
    for (;;) {
        { uint32_t now = HAL_GetTick(); Upper_Service(now); }
        osDelay(1U);
    }
}

/* 功能：执行 arm_set_command 函数对应的模块操作。 */
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

/* 功能：执行 Upper_SetJointMIT 函数对应的模块操作。 */
bool Upper_SetJointMIT(uint8_t arm, arm_joint_t joint, float tau_nm, float output_degrees)
{
/* 功能：执行 arm_set_command 函数对应的模块操作。 */
    return arm_set_command(arm, joint, ARM_MODE_MIT, output_degrees, tau_nm);
}

/* 功能：执行 Upper_SetJointModeTarget 函数对应的模块操作。 */
bool Upper_SetJointModeTarget(uint8_t arm, arm_joint_t joint, arm_mode_t mode, float target_degrees)
{
/* 功能：执行 arm_set_command 函数对应的模块操作。 */
    return arm_set_command(arm, joint, mode, target_degrees, 0.0f);
}

/* 功能：执行 Upper_SetJointDegrees 函数对应的模块操作。 */
bool Upper_SetJointDegrees(uint8_t arm, arm_joint_t joint, float degrees)
{
/* 功能：执行 arm_set_command 函数对应的模块操作。 */
    return arm_set_command(arm, joint, ARM_MODE_POSITION, degrees, 0.0f);
}

/* 功能：执行 Upper_QueueArmPositions 函数对应的模块操作。 */
void Upper_QueueArmPositions(bool enable)
{
    upper_state.enable = enable ? 1U : 0U;
    if (!enable)
        Arm_TaskStop();
}

/* 功能：执行 Arm_Task 函数对应的模块操作。 */
void Arm_Task(void *argument)
{
    uint32_t next = osKernelGetTickCount();
    (void)argument;
    for (;;) {
        Arm_ServiceGpio();
        next += (osKernelGetTickFreq() * 2U + 999U) / 1000U;
        if (osDelayUntil(next) != osOK)
            next = osKernelGetTickCount();
    }
}
