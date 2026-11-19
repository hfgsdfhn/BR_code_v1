#ifndef STATUS_TASK_H
#define STATUS_TASK_H

#include "remote.h"
#include "cmsis_os2.h"
#include "chassis.h"
#include <stdint.h>

struct send_to_chassis_t
{
    float dt35_right;
    float dt35_behind;
    remote_t remote;
    chassis_speed_t target_speed;
    float target_yaw;
    float current_yaw;
    uint8_t L2_up_flag;
    uint8_t j4_up_flag;
    uint8_t chassis_emmergency_stop;
    uint8_t leg_all_back;
};

typedef struct
{
    float dt35_right;
    float dt35_behind;
    remote_t remote;
    uint32_t sequence;
} send_to_arm_t;

typedef struct
{
    float dt35_right;
    float dt35_behind;
    remote_t remote;
    float stp23l_up_distance_cm;
    float stp23l_down_distance_cm;
    uint32_t sequence;
} send_to_cup_t;

typedef send_to_arm_t send_to_climb_t;

extern osMessageQueueId_t Chassis_QueueHandle;
extern osMessageQueueId_t Arm_QueueHandle;
extern osMessageQueueId_t Climb_QueueHandle;
extern osMessageQueueId_t Cup_QueueHandle;

extern volatile uint8_t gpio_pb6, gpio_pb7, gpio_pd7, gpio_pd6;
extern volatile uint8_t gpio_pd1, gpio_pd3, gpio_pd4, gpio_pd5;

void Status_InitGpio(void);
void Status_ApplyGpio(void);
void Queue_Init(void);
void Send_to_Chassis(void);
void Send_to_Arm(void);
void Send_to_Climb(void);
void Send_to_Cup(void);
void Status_Task(void *argument);

#endif
