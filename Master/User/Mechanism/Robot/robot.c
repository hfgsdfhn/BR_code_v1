#include "robot.h"
#include "bsp_can.h"
#include "board_can.h"
#include "upper.h"
#include "arm_task.h"
robot_state_t robot;

/* 功能：初始化Robot_Init对应的外设、参数和运行状态。 */
void Robot_Init(void)
{
    robot = ROBOT_STOP;

}

/* 功能：停止Robot_Stop对应的输出并清理运行状态。 */
void Robot_Stop(void)
{
    robot = ROBOT_STOP;
    Arm_TaskStop();
}
