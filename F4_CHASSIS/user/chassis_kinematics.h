#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>

typedef struct {
    float vx;
    float vy;
    float vw;
} chassis_speed_t;   // 底盘三轴速度

typedef struct {
    float lf;
    float rf;
    float lr;
    float rr;
} wheel_speed_t;     // 四轮轮速

typedef struct
{
    float fx;     
    float fy;  
    float mz;    
} chassis_force_t;  // 底盘三轴力
void Chassis_InverseKinematics(const chassis_speed_t *target,
                               float motor_speed[4]);

void Chassis_ForwardKinematics(const float motor_speed[4],
                                chassis_speed_t *actual);

void Chassis_ForceInverseKinematics(const chassis_force_t *target,
                                    float motor_torque[4]);

#endif