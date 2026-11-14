#include "chassis.h"
#include <stddef.h>
#include "math.h"
static float joystick_axis_to_unit(int16_t value)
{
    if (value > 100) value = 100;
    if (value < -100) value = -100;
    if(abs(value)<20) value = 0;
    return (float)value / 100.0f;
}

static float T_Speed_Control(float current, float target, float step)
{
    if (current < target)
    {
        current += step;

        if (current > target)
        {
            current = target;
        }
    }
    /* 减速 */
    else if (current > target)
    {
        current -= step;

        if (current < target)
        {
            current = target;
        }
    }

    return current;
}
void joy_to_mps(const remote_t *remote, chassis_speed_t *speed)
{
    float target_vx;
    float target_vy;
    float target_vw;

    if (speed == NULL)
    {
        return;
    }

    if ((remote == NULL) || (remote->online == 0U))
    {
        speed->vx = 0.0f;
        speed->vy = 0.0f;
        speed->vw = 0.0f;
        return;
    }
    
    target_vx = joystick_axis_to_unit(remote->joy_x) * CHASSIS_JOYSTICK_MAX_LINEAR_MPS;
    target_vy = joystick_axis_to_unit(remote->joy_y) * CHASSIS_JOYSTICK_MAX_LINEAR_MPS;
    if(remote->shoulder_left ==1) speed->vw = CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S;
    else if(remote->shoulder_left ==3) speed->vw = -CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S;
    else speed->vw = 0.0f;
    
    speed->vx = T_Speed_Control(speed->vx, target_vx, CHASSIS_LINEAR_SPEED_STEP_MPS);
    speed->vy = T_Speed_Control(speed->vy, target_vy, CHASSIS_LINEAR_SPEED_STEP_MPS);
    speed->vw = T_Speed_Control(speed->vw, target_vw, CHASSIS_ANGULAR_SPEED_STEP_RAD_S);
}

