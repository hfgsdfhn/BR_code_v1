#include "status_task.h"
#include "bsp_can.h"
#include "my_main.h"
#include "cup_task.h"
#include <string.h>

osMessageQueueId_t Chassis_QueueHandle;
osMessageQueueId_t Arm_QueueHandle;
osMessageQueueId_t Climb_QueueHandle;
osMessageQueueId_t Cup_QueueHandle;

volatile uint8_t gpio_pb6 = 0U;
volatile uint8_t gpio_pb7 = 0U;
volatile uint8_t gpio_pd7 = 0U;
volatile uint8_t gpio_pd6 = 0U;
volatile uint8_t gpio_pd1 = 0U;
volatile uint8_t gpio_pd3 = 0U;
volatile uint8_t gpio_pd4 = 0U;
volatile uint8_t gpio_pd5 = 0U;

static uint32_t status_sequence;
static chassis_speed_t planned_speed;

typedef struct
{
    float dt35_right;
    float dt35_behind;
    remote_t remote;
    chassis_speed_t target_speed;
    imu_t imu;
    float stp23l_up_distance_cm;
    float stp23l_down_distance_cm;
    uint32_t sequence;
} status_common_t;

static status_common_t common;

void Status_InitGpio(void)
{
    gpio_pb6 = 0U; gpio_pb7 = 0U; gpio_pd7 = 0U; gpio_pd6 = 0U;
    gpio_pd1 = 0U; gpio_pd3 = 0U; gpio_pd4 = 0U; gpio_pd5 = 0U;
}

void Status_ApplyGpio(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, gpio_pb6 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, gpio_pb7 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, gpio_pd7 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, gpio_pd6 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, gpio_pd1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, gpio_pd3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, gpio_pd4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, gpio_pd5 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Queue_Init(void)
{
    Chassis_QueueHandle = osMessageQueueNew(1U, sizeof(send_to_chassis_t), NULL);
    Arm_QueueHandle = osMessageQueueNew(1U, sizeof(send_to_arm_t), NULL);
    Climb_QueueHandle = osMessageQueueNew(1U, sizeof(send_to_climb_t), NULL);
    Cup_QueueHandle = osMessageQueueNew(1U, sizeof(send_to_cup_t), NULL);
}

static void Status_ReadAll(void)
{
    memset(&common, 0, sizeof(common));
    (void)Remote_GetState(&common.remote);
    joy_to_mps(&common.remote, &planned_speed);
    common.target_speed = planned_speed;
    common.imu = g_imu;
    common.stp23l_up_distance_cm = ustp23l_distance_cm;
    common.stp23l_down_distance_cm = dstp23l_distance_cm;
    common.sequence = ++status_sequence;
}

static void Queue_PublishLatest(osMessageQueueId_t queue, const void *message)
{
    uint8_t discarded[sizeof(send_to_chassis_t)];
    if (queue == NULL || message == NULL)
        return;
    if (osMessageQueuePut(queue, message, 0U, 0U) != osOK &&
        osMessageQueueGet(queue, discarded, NULL, 0U) == osOK)
        (void)osMessageQueuePut(queue, message, 0U, 0U);
}

void Send_to_Chassis(void)
{
    send_to_chassis_t chassis;
    chassis.dt35_right = common.dt35_right;
    chassis.dt35_behind = common.dt35_behind;
    chassis.remote = common.remote;
    chassis.target_speed = common.target_speed;
    chassis.imu = common.imu;
    chassis.sequence = common.sequence;
    Queue_PublishLatest(Chassis_QueueHandle, &chassis);
}

void Send_to_Arm(void)
{
    send_to_arm_t arm;
    arm.dt35_right = common.dt35_right;
    arm.dt35_behind = common.dt35_behind;
    arm.remote = common.remote;
    arm.sequence = common.sequence;
    Queue_PublishLatest(Arm_QueueHandle, &arm);
}

void Send_to_Climb(void)
{
    send_to_climb_t climb;
    climb.dt35_right = common.dt35_right;
    climb.dt35_behind = common.dt35_behind;
    climb.remote = common.remote;
    climb.sequence = common.sequence;
    Queue_PublishLatest(Climb_QueueHandle, &climb);
}

void Send_to_Cup(void)
{
    send_to_cup_t cup;
    cup.dt35_right = common.dt35_right;
    cup.dt35_behind = common.dt35_behind;
    cup.remote = common.remote;
    cup.stp23l_up_distance_cm = common.stp23l_up_distance_cm;
    cup.stp23l_down_distance_cm = common.stp23l_down_distance_cm;
    cup.sequence = common.sequence;
    Queue_PublishLatest(Cup_QueueHandle, &cup);
}

void Status_Task(void *argument)
{
    (void)argument;
    uint32_t tick = osKernelGetTickCount();
    static bool can1_bus_off_prev;
    static bool can2_bus_off_prev;
    static bool can3_bus_off_prev;

    for (;;)
    {
        bool can1_bus_off = ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U);
        bool can2_bus_off = ((hfdcan2.Instance->PSR & FDCAN_PSR_BO) != 0U);
        bool can3_bus_off = ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) != 0U);
        if (can1_bus_off && !can1_bus_off_prev) can1_diag.bus_off_count++;
        if (can2_bus_off && !can2_bus_off_prev) can2_diag.bus_off_count++;
        if (can3_bus_off && !can3_bus_off_prev) can3_diag.bus_off_count++;
        can1_bus_off_prev = can1_bus_off;
        can2_bus_off_prev = can2_bus_off;
        can3_bus_off_prev = can3_bus_off;
        BspCan_Service();
        Status_ReadAll();
        Send_to_Chassis();
        Send_to_Arm();
        Send_to_Climb();
        Send_to_Cup();
        Status_ApplyGpio();
        tick += 2U;
        osDelayUntil(tick);
    }
}
