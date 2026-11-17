#ifndef STATUS_TASK_H
#define STATUS_TASK_H

#include "mins_imu.h"
#include "remote.h"
#include "cmsis_os2.h"
#include "chassis.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    STATUS_CONTROL_OFFLINE = 0U,
    STATUS_CONTROL_REMOTE = 1U
} status_control_t;

typedef struct
{
    remote_t remote;
    chassis_speed_t target_speed;
    status_control_t control;
    uint32_t sequence;
} status_snapshot_t;

extern volatile status_snapshot_t g_status_snapshot;

void Status_Task(void *argument);
void Status_InitGpio(void);
void Status_ApplyGpio(void);
void Status_UpdateSnapshot(void);
bool Status_GetSnapshot(status_snapshot_t *snapshot);

#endif