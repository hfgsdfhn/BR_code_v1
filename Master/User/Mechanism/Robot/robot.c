#include "robot.h"
#include "bsp_can.h"
#include "board_can.h"
#include "upper.h"
#include "arm_task.h"
robot_state_t robot;

void Robot_Init(void)
{
    robot = ROBOT_STOP;

}

void Robot_Stop(void)
{
    robot = ROBOT_STOP;
    Arm_TaskStop();
}
