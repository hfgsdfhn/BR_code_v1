




#ifndef TRAPEZOID_PLANNER_H
#define TRAPEZOID_PLANNER_H

#include <stdint.h>


typedef struct
{
    float speed;
    float accel;
    float a_acc;
    float a_dec;
} trapezoid_t;


typedef struct
{
    float vx;
    float vy;
    float a_acc;
    float a_dec;
} trapezoid_2d_t;


void Trapezoid_Init(trapezoid_t *tp, float a_acc, float a_dec);


void Trapezoid_Reset(trapezoid_t *tp, float speed);


float Trapezoid_Update(trapezoid_t *tp, float target, float dt);


void Trapezoid2D_Init(trapezoid_2d_t *tp, float a_acc, float a_dec);


void Trapezoid2D_Reset(trapezoid_2d_t *tp, float vx, float vy);


void Trapezoid2D_Update(trapezoid_2d_t *tp, float tx, float ty, float dt, float *vx, float *vy);


float Trapezoid_BrakeSpeedLimit(float remaining, float a_dec);

#endif
