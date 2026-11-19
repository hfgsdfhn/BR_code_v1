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

float chassis_pid[2][4] = {
    {6.0f, 0.0f, 0.0f,6.0f},
    {4.0f, 0.0f, 0.0f, 5.0f}
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
    PID_Init(&yaw_pid, chassis_pid[0][0], chassis_pid[0][1], chassis_pid[0][2],chassis_pid[0][3], 1.0f);
    PID_Init(&turn_pid, chassis_pid[1][0], chassis_pid[1][1],chassis_pid[1][2],chassis_pid[1][3], 1.0f);
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
        float target_yaw_step = 0.001;

        if (send->remote.shoulder_left == 1U)
        {
            send->target_yaw -= target_yaw_step;
        }
        else if (send->remote.shoulder_left == 3U)
        {
            send->target_yaw += target_yaw_step;
        }
        float yaw_error = fabsf(send->target_yaw - send->current_yaw);
        if(yaw_error < 1.57f && yaw_error >0.001f) target_vw = -PID_Calc(&yaw_pid, send->target_yaw, send->current_yaw,0.002f);
        else if(yaw_error >=1.57f)                 target_vw = -PID_Calc(&turn_pid, send->target_yaw, send->current_yaw,0.002f);
        else target_vw = 0.0f;
        send->target_speed.vw = T_Speed_Control(send->target_speed.vw, target_vw, 0.08f);
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
           send->target_speed.vw = T_Speed_Control(send->target_speed.vw, target_vw,
                                             CHASSIS_ANGULAR_SPEED_STEP_RAD_S);
    }
    send->target_speed.vx = T_Speed_Control(send->target_speed.vx, target_vx,
                                             CHASSIS_LINEAR_SPEED_STEP_MPS);
    send->target_speed.vy = T_Speed_Control(send->target_speed.vy, target_vy,
                                             CHASSIS_LINEAR_SPEED_STEP_MPS);
}

void key_to_chassis(send_to_chassis_t *send)
{
    static uint8_t l2_up_flag;
    static uint8_t j4_flag;
    static uint8_t chassis_emmergency_stop;
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
        M2006_wheel_ctrl(pe1_pressed ? -CHASSIS_WHEEL_JOG_SPEED_MPS * persent
                                    : CHASSIS_WHEEL_JOG_SPEED_MPS * persent);
    }

    if (key_is_pressed(REMOTE_KEY_PE6))
    {
        l2_up_flag = (uint8_t)((l2_up_flag + 1U) % 7U);
    }
    send->L2_up_flag = l2_up_flag;

    if (key_is_pressed(REMOTE_KEY_PE3))
    {
        j4_flag++;
        if(j4_flag > 5) j4_flag = 3;
        send->j4_up_flag = j4_flag;
    }
    if (key_is_pressed(REMOTE_KEY_PE5))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE4))
    {
        send->chassis_emmergency_stop = !chassis_emmergency_stop;
    }
    if (key_is_pressed(REMOTE_KEY_PE2))
    {
        j4_flag = 0;
        l2_up_flag = 0;
    }
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
            DJI_Board_Target.leg_height_mm[0] = posa +front; 
            DJI_Board_Target.leg_height_mm[1] = posa +behind; 
            DJI_Board_Target.leg_height_mm[2] = posa +behind; 
            DJI_Board_Target.leg_height_mm[3] = posa +front; 
    Send_Target_Pos(&hfdcan1, DJI_Board_Target.leg_height_mm);
}
void l2_leg_up(L2_Leg_up_e leg_up)
{
    switch (leg_up)
    {
    case l2_up_all_go:
        set_target_position(320.0f, 0.0f, 9.6f);
        break;
    case l2_up_front_back:
        set_target_position(320.0f, -320.0f, 9.6f);
        break;
    case l2_up_behind_back:
        set_target_position(0.0f,0.0f,0.0f);
        break;
    case l2_down_behind_go:
        set_target_position(0.0f, 0.0f, 329.6f);
        break;
    case l2_down_front_go:
        set_target_position(320.0f, 0.0f, 9.6f);
        break;
    case l2_down_all_back:
        set_target_position(0.0f, 0.0f, 0.0f);
    default:
        set_target_position(0.0f,0.0f,0.0f);
        break;
    }
}

void j4_up(J4_up_e leg_up)
{
    switch (leg_up)
    {
    case j4_all_go:
        set_target_position(170.0f,0.0f,9.6f);
        break;
    case j4_front_back:
        set_target_position(0.0f,0.0f,179.6f);
        break;
    case j4_go_again:
        gpio_pd3 = 0;
        set_target_position(170.0f,0.0f,179.6f);
        break;
    case j4_front_back_qigang_go:
        gpio_pd3 = 1;
        set_target_position(0.0f,0.0f,349.6f);
        break;
    case j4_behind_back:
        set_target_position(0.0f,0.0f,140.0f);
        break;
    default:
        set_target_position(0.0f,0.0f,0.0f);
        break;
    }
}
