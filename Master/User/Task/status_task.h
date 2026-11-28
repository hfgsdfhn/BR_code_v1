#ifndef STATUS_TASK_H
#define STATUS_TASK_H

#include "remote.h"
#include "cmsis_os2.h"
#include "dt35.h"
#include "chassis.h"
#include <stdint.h>

#define GPIO_INPUT_COUNT 5

typedef struct
{
    float dt35_right;
    float dt35_front;
    remote_t remote;
    chassis_speed_t target_speed;
    chassis_speed_t current_speed;
    chassis_walk_t chassis_walk;
    float target_yaw;
    float current_yaw;
    float stp23l_up_distance_cm;
    float stp23l_down_distance_cm;
    uint32_t sequence;
} status_common_t;

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vw;
    int16_t yaw;
    int16_t dt35_left;
    int16_t dt35_front;
    uint8_t action_task;       //用来下任务 暂时先空发
    uint8_t remote_state;      //遥控器是否在线
}send_to_ethernet_t;

struct send_to_chassis_t
{
    float dt35_right;
    float dt35_front;
    remote_t remote;
    chassis_speed_t target_speed;
    chassis_walk_t chassis_walk;
    chassis_speed_t current_speed;
    float leg_wheel_speed;
    float target_yaw;
    float current_yaw;
    uint8_t leg_up_flag;
    uint8_t leg_back_flag;
    uint8_t chassis_emmergency_stop;
};


typedef struct
{
    float dt35_right;
    float dt35_front;
    remote_t remote;
    uint32_t sequence;
} send_to_arm_t;

typedef struct
{
    float dt35_right;
    float dt35_front;
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
extern volatile uint8_t gpio_pa8, gpio_pc9, gpio_pc8;
extern uint8_t gpio_pa8_flag;
extern uint8_t gpio_pc8_flag;
extern uint8_t gpio_pc9_flag;
void Status_InitGpio(void);
void Status_ApplyGpio(void);
void Queue_Init(void);
void Send_to_Chassis(void);
void Send_to_Arm(void);
void Send_to_Climb(void);
void Send_to_Cup(void);
void Status_Task(void *argument);

#endif
