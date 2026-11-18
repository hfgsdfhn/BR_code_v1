#include "chassis.h"
#include "status_task.h"
#include "mins_imu.h"
#include <stddef.h>
#include <stdlib.h>
#include "chassis_task.h"
#include "board_can.h"
#include "math.h"
#include "pid.h"
 
#define use_imu 1

#define CHASSIS_WHEEL_JOG_SPEED_MPS 0.5f
float leg_pos = 100.0f;
pid_t yaw_pid;
pid_t turn_pid;
float persent = 0.0f;

float chassis_pid[2][3] = {
    {6.1f, 0.0f, 5.0f},
    {3.0f, 0.0f, 1.0f}
	};
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

void chassis_init()
{
    init_board();
    PID_Init(&yaw_pid, chassis_pid[0][0], chassis_pid[0][1], chassis_pid[0][2],10.0f, 0.0f);
    PID_Init(&turn_pid, chassis_pid[1][0], chassis_pid[1][1],chassis_pid[1][2],5.0f, 0.0f);
}


void joy_to_mps(send_to_chassis_t *send)
{
    float target_vx;
    float target_vy;
    float target_vw;

    if (send == NULL)
    {
        return;
    }

    if (send->remote.online == 0U)
    {
        send->target_speed.vx = 0.0f;
        send->target_speed.vy = 0.0f;
        send->target_speed.vw = 0.0f;
        return;
    }
    
    target_vx = joystick_axis_to_unit(send->remote.joy_x) * CHASSIS_JOYSTICK_MAX_LINEAR_MPS;
    target_vy = joystick_axis_to_unit(send->remote.joy_y) * CHASSIS_JOYSTICK_MAX_LINEAR_MPS;
    if (use_imu)
    {
        float target_yaw_step = CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S
                              * CHASSIS_CONTROL_PERIOD_S
                              * CHASSIS_RAD_TO_DEG;

        if (send->remote.shoulder_left == 1U)
        {
            send->target_yaw -= target_yaw_step;
        }
        else if (send->remote.shoulder_left == 3U)
        {
            send->target_yaw += target_yaw_step;
        }
        float yaw_error = fabsf(send->target_yaw - send->current_yaw);
        if(yaw_error < 0.5f && yaw_error >0.02f) target_vw = -PID_Calc(&yaw_pid, send->target_yaw, send->current_yaw,CHASSIS_CONTROL_PERIOD_S);
        else if(yaw_error >=0.5f)                target_vw = -PID_Calc(&turn_pid, send->target_yaw, send->current_yaw,CHASSIS_CONTROL_PERIOD_S);
        else target_vw = 0.0f;
    }
    else
    {
        if (send->remote.shoulder_left == 1U)
        {
            target_vw = CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S;
        }
        else if (send->remote.shoulder_left == 3U)
        {
            target_vw = -CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S;
        }
        else
        {
            target_vw = 0.0f;
        }
    }
    send->target_speed.vx = T_Speed_Control(send->target_speed.vx, target_vx,
                                             CHASSIS_LINEAR_SPEED_STEP_MPS);
    send->target_speed.vy = T_Speed_Control(send->target_speed.vy, target_vy,
                                             CHASSIS_LINEAR_SPEED_STEP_MPS);
    send->target_speed.vw = T_Speed_Control(send->target_speed.vw, target_vw,
                                             CHASSIS_ANGULAR_SPEED_STEP_RAD_S);
}

void key_to_chassis(send_to_chassis_t *send)
{
    uint8_t pe0_pressed;
    uint8_t pe1_pressed;

    if (send == NULL)
    {
        return;
    }

    pe0_pressed = (uint8_t)((send->remote.key[1] & (1U << 7)) != 0U);
    pe1_pressed = (uint8_t)((send->remote.key[1] & (1U << 6)) != 0U);
    if (send->remote.online == 0U || pe0_pressed == pe1_pressed)
    {
        M2006_wheel_ctrl(0.0f);
        persent = 0.0f;
    }
    else
    {
        persent += 0.001f;
        if(persent >1.0f) persent = 1.0f;
        M2006_wheel_ctrl(pe1_pressed ? CHASSIS_WHEEL_JOG_SPEED_MPS * persent
                                    : -CHASSIS_WHEEL_JOG_SPEED_MPS * persent);
    }

    if ((send->remote.online == 0U) || (g_imu.update_count == 0U))
    {
        return;
    }

    // if (key_is_pressed(REMOTE_KEY_PE6))
    // {
    //     leg_target_pos_a +=  leg_pos;     //一块升

    // }
    // if (key_is_pressed(REMOTE_KEY_PE3))
    // {
    //     leg_target_pos_f -=  leg_pos;     //前腿收
    // }
    // if (key_is_pressed(REMOTE_KEY_PE5))
    // {
    //     leg_target_pos_b -=  leg_pos;    //后退收 
    // }
    // if (key_is_pressed(REMOTE_KEY_PE4))
    // {
    //     gpio_pd3 != gpio_pd3;
    // }
    // if (key_is_pressed(REMOTE_KEY_PE2))
    // {
    //     leg_target_pos_a -=  leg_pos;
    // }

    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_1))
    {
        send->target_yaw += 1.5708f;
    }
    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_2))
    {
        send->target_yaw += 3.1415f;
    }
    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_3))
    {  
        send->target_yaw -= 1.5708f;
    }
}

void M2006_wheel_ctrl(float speed)
{
    for(int i = 0; i <4 ; i++)
    {
        DJI_Board_Target.wheel_speed_mps[i] = speed;
    }
    Send_Target_Speed(&hfdcan1, DJI_Board_Target.wheel_speed_mps);
}

void set_target_position(float posa,float front, float behind)
{
            DJI_Board_Target.leg_height_mm[0] = posa +behind; 
            DJI_Board_Target.leg_height_mm[1] = posa +front; 
            DJI_Board_Target.leg_height_mm[2] = posa +front; 
            DJI_Board_Target.leg_height_mm[3] = posa +behind; 
    Send_Target_Pos(&hfdcan1, DJI_Board_Target.leg_height_mm);
}
