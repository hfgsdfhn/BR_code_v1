





#ifndef MECHANISM_ROBOT_H
#define MECHANISM_ROBOT_H



typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_RUN
} robot_state_t;

extern robot_state_t robot;


void Robot_Init(void);


void Robot_Stop(void);

#endif
