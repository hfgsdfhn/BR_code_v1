/*
 * @file Chassis_Task.c
 * @brief FreeRTOS 底盘控制任务。
 */

#include "Chassis_Task.h"
#include "Auto_Task.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "Task.h"
#include "path.h"
#include "DT35_process.h"
#include "visual_process.h"
#include "small_board_link.h"
#include "cmsis_os2.h"
#include "dm_imu.h"
#include "kalman.h"
#include "path_follow.h"
#include "fdcan.h"
#include "pid.h"
#include <math.h>
#include <string.h>
#include "robot.h"

/*------------------ 底盘速度限制 ----------------------*/
static void Chassis_Auto(void);
static void remote_process(void);
static void Chassis_Task_Control(void);
static float Speed_Control(float current, float target, float v_max, float acceleration, float dt);   //实现电机速度步进，并且对最终速度进行限制
static bool stop = 0;               //锁底盘
static bool auto_started = false;   
static uint8_t last_left_keys = 0;   // 上一次的稳定状态
static uint8_t last_sequence = 0;
static bool sequence_initialized = false;

float x = -0.01f;
float y = 0.0f;

uint8_t dt35_path_state = 0;

ak_motor_t ak_wheel[4];
pid_t imu_pid;
pid_t turn_pid;
path_follow_t follower;
path_follow_speed_t speed;

void chassis_init(void)
{
    AkMotor_Init(&ak_wheel[wheel_lf], &hfdcan1, 0x03);
    AkMotor_Init(&ak_wheel[wheel_rf], &hfdcan1, 0x04);
    AkMotor_Init(&ak_wheel[wheel_lr], &hfdcan1, 0x01);
    AkMotor_Init(&ak_wheel[wheel_rr], &hfdcan1, 0x02);
    DM_IMU_Init(0);
    PID_Init(&imu_pid, 6.0f, 0.0f, 5.0f, 2.0f, 10.0f, 0.0f);
    PID_Init(&turn_pid, 3.0f, 0.0f, 1.0f, 2.0f, 5.0f, 0.0f);
    PID_Init(&dt35_front_pid, 2.2f, 0.001f, 1.8f,2.0f,DT35_FRONT_PID_LIMIT,0.0f);
    PID_Init(&dt35_left_pid, 2.3f, 0.01f, 1.0f,2.0f,DT35_LEFT_PID_LIMIT,0.1f);
    PathFollow_Init(&follower,
                    path,
                    PATH_SIZE,
                    150.0f, /* 前视距离，单位 mm。 */
                    1.5f,   /* 法向纠偏增益。 */
                    1.0f,   /* 法向速度上限，单位 m/s。 */
                    4.0f,   /* yaw 增益。 */
                    3.0f,   /* yaw 速度上限，单位 rad/s。 */
                    50.0f); /* 终点停止距离，单位 mm。 */
    PID_SetIntegralSeparation(&imu_pid, 40.0f * 0.01745f);
    Kalman_Init(&filter_x, 0.0f, 1.0f, Kalman_Q, Kalman_R);
    Kalman_Init(&filter_y, 0.0f, 1.0f, Kalman_Q, Kalman_R);
    Path_Init(&pos);
}

void chassis_stop(void)
{
    uint8_t index;

    memset(&chassis, 0, sizeof(chassis));

    for (index = 0U; index < 4U; index++)
    {
        ak_wheel[index].command_speed = 0.0f;
        AkMotor_SetSpeed(&ak_wheel[index], 0.0f);

    }
}

void speed_to_chassis(void)
{
    chassis.state.wheel.lf = ak_wheel[wheel_lf].feedback.speed * ERPM_TO_MPS;
    chassis.state.wheel.rf = ak_wheel[wheel_rf].feedback.speed * ERPM_TO_MPS;
    chassis.state.wheel.lr = ak_wheel[wheel_lr].feedback.speed * ERPM_TO_MPS;
    chassis.state.wheel.rr = ak_wheel[wheel_rr].feedback.speed * ERPM_TO_MPS;
}

void speed_to_motor(void)
{
    ak_wheel[wheel_lf].command_speed = chassis.target_wheel.lf;
    ak_wheel[wheel_rf].command_speed = chassis.target_wheel.rf;
    ak_wheel[wheel_lr].command_speed = chassis.target_wheel.lr;
    ak_wheel[wheel_rr].command_speed = chassis.target_wheel.rr;
}

static float apply_deadzone(float value, float deadzone)
{
    if (value < deadzone && value > -deadzone)
    {
        return 0.0f;
    }
    return value;
}

/*--------------------------- FreeRTOS 底盘控制线程入口。-----------------------------*/
void StartChassisTask(void *argument /* FreeRTOS 创建底盘任务时传入的上下文。 */)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Chassis_Task_T);
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //底盘
    if(!stop)
    {
        //控制源
   //     remote_process();          //遥控
        Chassis_Auto();     //自动
        //任务执行
        Chassis_Task_Control();
    }  else 

    {
        chassis_stop();
   //     remote_process();
    }
    }
}

//0不启动世界坐标系，1启动（暂时没加）
void Chassis_Task_Control(void)
{

    //缓启动
    chassis.command_speed.vx = Speed_Control(chassis.last_speed.vx, chassis.target_speed.vx, V_MAX, V_STEP, Chassis_Task_T);
    chassis.command_speed.vy = Speed_Control(chassis.last_speed.vy, chassis.target_speed.vy, V_MAX, V_STEP, Chassis_Task_T);
    //旋转
    if(!yaw_correct)    
    {
        chassis.command_speed.vw = Speed_Control(chassis.last_speed.vw, chassis.target_speed.vw, V_MAX, V_STEP, Chassis_Task_T);
    }
    if(yaw_correct)     //yaw旋转
    {   
        float yaw_feedback = chassis.state.position.yaw;
        //限制目标角度
        while (chassis.target_position.yaw >= 3.1415926f) chassis.target_position.yaw -= 6.283f;
        while (chassis.target_position.yaw < -3.1415926f) chassis.target_position.yaw += 6.283f;

        float yaw_target = chassis.target_position.yaw;
        float yaw_error = yaw_target - yaw_feedback;

        // 选择跨越 -PI/PI 边界时的最短旋转路径。
        while (yaw_error >= 3.1415926f) yaw_error -= 6.2831852f;
        while (yaw_error < -3.1415926f) yaw_error += 6.2831852f;

        //yaw角pid计算
        if(fabsf(yaw_error) <= 0.5f && fabsf(yaw_error) >= 0.02f)
        chassis.command_speed.vw =  -PID_Calc(&imu_pid, yaw_error, 0);
        else if(fabsf(yaw_error) > 0.5f)
        chassis.command_speed.vw =  -PID_Calc(&turn_pid, yaw_error, 0);    //旋转时的pid
        else chassis.command_speed.vw = 0.0f;
    }
        chassis.last_speed.vx = chassis.command_speed.vx;
        chassis.last_speed.vy = chassis.command_speed.vy;
        chassis.last_speed.vw = chassis.command_speed.vw;

    if(world_system)
    {
        speed_to_chassis();  //从电机获取速度并转化单位
        chassis_get_speed();  //获取底盘真实速度
        Chassis_BodyToWorld();
    }
    chassis_set_speed();//底盘解算
    speed_to_motor();      //控制速度处理后发给电机结构体

    //预测值更新：将底盘机体系速度转换到世界坐标系
    {
        float cos_yaw = cosf(chassis.state.position.yaw);
        float sin_yaw = sinf(chassis.state.position.yaw);
        float world_vx = chassis.state.speed.vx * cos_yaw -
                         chassis.state.speed.vy * sin_yaw;
        float world_vy = chassis.state.speed.vx * sin_yaw +
                         chassis.state.speed.vy * cos_yaw;

        kalman_predict(&filter_x, world_vx, 2.0f);
        kalman_predict(&filter_y, world_vy, 2.0f);
    }

    //雷达数据更新时观测值更新

    if ((!sequence_initialized ||
        visua.robot_sequence != last_sequence))
    {
        kalman_update(&filter_x, visua.robot_x_m);
        kalman_update(&filter_y, visua.robot_y_m);

        last_sequence = visua.robot_sequence;
        sequence_initialized = true;
    }

    for (uint8_t index = 0U; index < 4U; index++)//速度发给底盘
    {
       AkMotor_SendServoSpeedCommand(&ak_wheel[index]);
    }
}

static void Chassis_Auto(void)
{
    // if (auto_mode == AUTO_MODE_AUTO)
    // {
    //     if (!auto_started)
    //     {
    //         dt35_path_state = GO_PATH_1;
    //         auto_started = true;
    //     }

    //     else Path_Task();
    // }
    // else
    // {
    //     auto_started = false;
    // }
    if(auto_mode == leida)      //雷达追踪
    {
        if(follower.finished == false)
        {
                PathFollow_Calc(&follower,
                filter_x.x,
                filter_y.x,
                chassis.state.position.yaw,
                &speed);

        chassis.target_speed.vx = -speed.vx;
        chassis.target_speed.vy = -speed.vy;
        }

        if(follower.finished == true)
        {
            if(visualHandle != NULL)
            {
                chassis.target_speed.vx = 0.0f;
                chassis.target_speed.vy = 0.0f; 
            }
            if (osSemaphoreAcquire(visualHandle, 1) == osOK && visualHandle != NULL)  //等待视觉传来信息
            {
                visua.target_x_m = filter_x.x + visua.block_x_m + x;
                visua.target_y_m = filter_y.x + visua.block_y_m + y;
                osSemaphoreDelete(visualHandle);   // 用完销毁
                visualHandle = NULL;
            }
            if(visualHandle == NULL)
            {
                chassis.target_speed.vx = -PID_Calc(&dt35_left_pid, visua.target_x_m, filter_x.x);
                if(fabsf(visua.target_x_m - filter_x.x) <= 0.03f)
                chassis.target_speed.vy = -PID_Calc(&dt35_front_pid, visua.target_y_m, filter_y.x);                
            }
        }
    }

    if(auto_mode == duikuai)
    {
        follower.finished = true;
            if(visualHandle != NULL)
            {
                chassis.target_speed.vx = 0.0f;
                chassis.target_speed.vy = 0.0f; 
            }
            if (osSemaphoreAcquire(visualHandle, 1) == osOK && visualHandle != NULL)  //等待视觉传来信息
            {
                float world_x = 0.0f;
                float world_y = 0.0f;
                world_x = filter_x.x - visua.block_x_m + x;
                world_y = filter_y.x - visua.block_y_m + y;
                chassis.target_position.yaw = -visua.block_yaw;
                float cos_yaw = cosf(-chassis.state.position.yaw);
                float sin_yaw = sinf(-chassis.state.position.yaw);
                visua.target_x_m = cos_yaw *world_x + sin_yaw *world_y;
                visua.target_y_m = cos_yaw *world_y - sin_yaw *world_x;
                osSemaphoreDelete(visualHandle);   // 用完销毁
                visualHandle = NULL;
            }
            if(visualHandle == NULL)
            {
                float world_x = 0.0f;
                float world_y = 0.0f;
                float cos_yaw = cosf(-chassis.state.position.yaw);
                float sin_yaw = sinf(-chassis.state.position.yaw);
                world_x = cos_yaw*filter_x.x + sin_yaw*filter_y.x;
                world_y = cos_yaw*filter_y.x - sin_yaw*filter_x.x;
                chassis.target_speed.vx = -PID_Calc(&dt35_left_pid, visua.target_x_m, world_x);
                if(fabsf(visua.target_x_m - filter_x.x) <= 0.03f)
                chassis.target_speed.vy = -PID_Calc(&dt35_front_pid, visua.target_y_m, world_y);                
            }
    }
}

//imu数据处理
void ImuLink_OnData(const uint8_t *data, uint16_t size)
{
    DM_IMU_ParseFeedback(data, size);
    chassis.state.position.yaw = DM_IMU_GetYawFromSource(YAW_SOURCE_EULER);
}

//底盘遥控器处理
static void remote_process(void)
{
    remote_snapshot_t remote;       //读取遥控器状态
    if (!RemoteLink_GetSnapshot(&remote))
    {
        chassis.target_speed.vx = 0.0f;
        chassis.target_speed.vy = 0.0f;
        chassis.target_speed.vw = 0.0f;
        return;
    }

    //应用左摇杆
    chassis.target_speed.vx = apply_deadzone((remote.left_x - 128.0f) / 128.0f * V_MAX, 0.1f);
    chassis.target_speed.vy = apply_deadzone((remote.left_y - 128.0f) / 128.0f * V_MAX, 0.1f);

    switch (remote.shoulders)       //肩键控制旋转
    {
        case shoulder_left:
            if(yaw_correct)
                chassis.target_position.yaw -= 0.003f;
            else
                chassis.target_speed.vw = 1.2f;     //rad/s
         break;
        case shoulder_right:
            if(yaw_correct)
                chassis.target_position.yaw += 0.003f;
            else
                chassis.target_speed.vw = -1.2f;
         break;

        default:
            chassis.target_speed.vw = 0.0f;
            break;
    }

    //只有上次没有按键、这次有按键时，才认为是一次新的按下
    if (remote.left_keys != 0 && last_left_keys == 0)
    {
        switch (remote.left_keys)
        {
            case key_chassis_stop:
                stop = !stop;
                break;

            case left_90:
                chassis.target_position.yaw += 1.57f;
                break;

            case right_90:
                chassis.target_position.yaw -= 1.57f;
                break;

            default:
                break;
        }
    }
    last_left_keys = remote.left_keys;   // 更新历史状态
}


static float Speed_Control(float current, float target, float v_max, float acceleration, float dt)
{
    float step;

    /* 目标速度限幅 */
    if (target > v_max)
    {
        target = v_max;
    }
    else if (target < -v_max)
    {
        target = -v_max;
    }

    step = (acceleration * dt) / 1000.0f;

    /* 加速 */
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
