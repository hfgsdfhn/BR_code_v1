#include "robot.h"
#include "bsp_can.h"
#include "upper.h"
robot_state_t robot;
void Robot_Init(void)
{
    robot = ROBOT_STOP;
    Upper_Init(&upper_state);
}
void Robot_Stop(void)
{
    robot = ROBOT_STOP;
    Upper_Stop(&upper_state);
}
void BspCan_OnFrame(const can_frame_t *frame)
{
    Upper_OnFrame(&upper_state, frame);
}
