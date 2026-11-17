#include "chassis.h"
#include "mins_imu.h"
#include <stddef.h>
#include <stdlib.h>
#include "board_can.h"
#include "math.h"
#include "pid.h"

pid_t yaw_pid;

float chassis_pid[4] =
{1.0f, 0.0f, 0.0f, 1000.0f};

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
    PID_Init(&yaw_pid, chassis_pid[0], chassis_pid[1], chassis_pid[2],
             CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S, chassis_pid[3]);
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
    if (g_imu.update_count != 0U)
    {
        float target_yaw_step = CHASSIS_SHOULDER_MAX_ANGULAR_RAD_S
                              * CHASSIS_CONTROL_PERIOD_S
                              * CHASSIS_RAD_TO_DEG;

        if (send->remote.shoulder_left == 1U)
        {
            send->target_yaw += target_yaw_step;
        }
        else if (send->remote.shoulder_left == 3U)
        {
            send->target_yaw -= target_yaw_step;
        }

        target_vw = PID_Calc(&yaw_pid, send->target_yaw, send->current_yaw,
                             CHASSIS_CONTROL_PERIOD_S);
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
    if ((send == NULL) || (send->remote.online == 0U)
        || (g_imu.update_count == 0U))
    {
        return;
    }

    if (key_is_pressed(REMOTE_KEY_PE0))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE1))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE2))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE3))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE4))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE5))
    {
    }
    if (key_is_pressed(REMOTE_KEY_PE6))
    {
    }

    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_0))
    {
        
    }
    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_1))
    {
        send->target_yaw += 90;
    }
    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_2))
    {
        send->target_yaw += 180;
    }
    if (key_is_pressed(REMOTE_KEY_SHOULDER_RIGHT_3))
    {  
        send->target_yaw -= 90;
    }
}


void leg_M3508_ctrl(DJI_Board *dji_board,
                    DJI_PID_type_e pid_type,
                    uint8_t motor_id,
                    float ki,
                    float kd,
                    float target)
{
    uint8_t leg_index;
    uint8_t pid_index;

    if ((dji_board == NULL) ||
        (motor_id < M3508_LEG_MOTOR_ID_MIN) ||
        (motor_id > M3508_LEG_MOTOR_ID_MAX))
    {
        return;
    }

    pid_index = (uint8_t)pid_type;
    leg_index = (uint8_t)(motor_id - M3508_LEG_MOTOR_ID_MIN);

    dji_board->PID_K[pid_index][1] = ki;
    dji_board->PID_K[pid_index][2] = kd;
    dji_board->leg_height_mm[leg_index] = target;

    (void)SmallBoard_Send_PID(&hfdcan1,
                              pid_index,
                              dji_board->PID_K[pid_index][0],
                              ki,
                              kd);
    (void)Send_Target_Pos(&hfdcan1, dji_board->leg_height_mm);
}

void set_target_position(float posa,float front, float behind)
{
            DJI_Board_Target.leg_height_mm[0] = posa +behind; 
            DJI_Board_Target.leg_height_mm[1] = posa +front; 
            DJI_Board_Target.leg_height_mm[2] = posa +front; 
            DJI_Board_Target.leg_height_mm[3] = posa +behind; 
    Send_Target_Pos(&hfdcan1, DJI_Board_Target.leg_height_mm);
}
