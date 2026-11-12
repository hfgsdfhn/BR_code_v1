




#ifndef QUINTIC_PLANNER_H
#define QUINTIC_PLANNER_H

#include <stdint.h>


#define QUINTIC_PEAK_SEGMENTS 48U


#define QUINTIC_PEAK_MARGIN 1.02f


#define QUINTIC_K0_MIN (-2.0f)
#define QUINTIC_K0_MAX (2.5f)


#define QUINTIC_EPS 1.0e-6f

typedef struct
{
    float start;
    float delta;
    float k0;
    float duration;
    float v_scale;
    float peak_speed;
    float peak_accel;
    uint8_t limited;
} quintic_plan_t;


void Quintic_SetDefaultLimits(float max_speed, float max_accel, float min_duration);


void Quintic_Plan(quintic_plan_t *plan, float start, float target, float v0, float duration);


float Quintic_MinDuration(float delta, float v0);


void Quintic_Sample(const quintic_plan_t *plan, float elapsed, float *pos, float *vel, float *acc);


extern volatile float quintic_last_duration_s;
extern volatile float quintic_last_peak_speed;
extern volatile float quintic_last_peak_accel;
extern volatile uint8_t quintic_last_limited;

#endif
