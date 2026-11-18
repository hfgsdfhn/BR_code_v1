#include "my_main.h"
#include "board_can.h" // 引入小板解析出来的目标值
#include "can.h"
#include "math.h"
#include <stddef.h>

volatile uint8_t flag_motor_comm = 0; 
volatile uint8_t flag_100ms_comm = 0;
pid_t leg_position_pid[4];
pid_t leg_speed_pid[4];
pid_t wheel_speed_pid[4];
dji_motor_t M2006[4];
dji_motor_t M3508[4];
static const int8_t m2006_dir[4] = { -1,  -1,  1,  1 };
static const int8_t m3508_dir[4] = { 1, -1,  1, -1 };
float debug_target;
static void Calc_Height_Sync_Comp(dji_motor_t *motors[4], float target_height[4], float comp_out[4], float k_sync);
float pid_use(pid_t *pid, float kp, float ki, float kd, float target, float feedback);

float limit_abs(float value, float max)
{
    if (value > max) return max;
    if (value < -max) return -max;
    return value;
}

void My_Init(void)
{
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&leg_position_pid[i], PID_K[0][0], PID_K[0][1], PID_K[0][2], 5.0f, 2900.0f, 0.0f);
        PID_Init(&leg_speed_pid[i],    PID_K[1][0], PID_K[1][1], PID_K[1][2], 5.0f, 16000.0f, 2000.0f);
        PID_Init(&wheel_speed_pid[i],  PID_K[2][0], PID_K[2][1], PID_K[2][2], 5.0f, 10000.0f, 1000.0f);

        // 初始化电机 (CAN2)
        DjiMotor_Init(&M2006[i], &hcan2, i + 1U, DJI_MOTOR_TYPE_M2006, m2006_dir[i]);
        DjiMotor_Init(&M3508[i], &hcan2, i + 5U, DJI_MOTOR_TYPE_M3508, m3508_dir[i]);
    }
}

void My_Loop(void)
{
    static uint8_t m3508_zeroed = 0U;
    
    if (flag_motor_comm)
    {
        flag_motor_comm = 0;

        for (int i = 0; i < 4; i++)
        {
            if (M3508[i].encoder_ready == 0U)
            {
                return;
            }
        }

        if (m3508_zeroed == 0U)
        {
					
            DjiMotor_SetZero(M3508, 4U);
            for (int i = 0; i < 4; i++)
            {
                PID_Reset(&leg_position_pid[i]);
                PID_Reset(&leg_speed_pid[i]);
            }
            m3508_zeroed = 1U;
        }

        float h_ref, h_cur, v_ref, v_cur;
        float leg_out[4];
        float w_ref, w_cur;
        float whl_out[4];
        
        float sync_comp[4]; // 存放同步补偿速度

        /* 重力补偿参数 */
        static const float grav_comp_target[4] = { 5500.0f, 3200.0f, 1300.0f, 7000.0f }; 
        static const float grav_deadband_mm    = 10.0f;    
        static const float grav_full_mm        = 30.0f;  

        /* ================= 新增：调用封装函数计算同步补偿 ================= */
        static const float K_sync = 0.0f;  

        // 构建结构体指针数组，指向 M3508 的 4 个电机
        dji_motor_t *leg_motors[4] = { &M3508[0], &M3508[1], &M3508[2], &M3508[3] };
        float target_height_enc[4];
        for(int i = 0; i < 4; i++) {
            // 把目标高度(mm)转成脉冲数
            target_height_enc[i] = leg_height_mm[i] / encoder_to_mm; 
        }
        // 调用函数计算补偿
        Calc_Height_Sync_Comp(leg_motors, target_height_enc, sync_comp, K_sync);


        /* ================= PID 计算（for 循环） ================= */
        for (int i = 0; i < 4; i++)
        {
            /* 腿部位置环 */
            h_ref = leg_height_mm[i] / encoder_to_mm;
            h_cur = M3508[i].total_encoder; // 这里直接用结构体里的值
            v_ref = pid_use(&leg_position_pid[i], PID_K[0][0], PID_K[0][1],
                            PID_K[0][2], h_ref, h_cur);

            /* --- 使用同步补偿 --- */
            // 如果 sync_comp[i] > 0 (腿偏短跑慢了)，减去它相当于给正的加速指令
            v_ref += sync_comp[i]; 

            /* 腿部速度环 */
            v_cur = M3508[i].speed_rpm; // 结构体里的速度反馈
            leg_out[i] = pid_use(&leg_speed_pid[i], PID_K[1][0], PID_K[1][1],
                                PID_K[1][2], v_ref, v_cur);
            leg_out[i] = limit_abs(leg_out[i], 16000.0f);

            /* 重力补偿 */
            {
                float ah = fabsf(h_ref);
                float ratio = (ah > grav_deadband_mm) ?
                            (ah - grav_deadband_mm) / (grav_full_mm - grav_deadband_mm) : 0.0f;
                if (ratio > 1.0f) ratio = 1.0f;
                leg_out[i] += grav_comp_target[i] * ratio;
            }
            leg_out[i] = limit_abs(leg_out[i], 16000.0f);

            /* 轮子速度环 */
            w_ref = wheel_speed_mps[i] / RPM_TO_SPEED;
            w_cur = M2006[i].speed_rpm;
            whl_out[i] = pid_use(&wheel_speed_pid[i], PID_K[2][0], PID_K[2][1],
                PID_K[2][2], w_ref, w_cur);
                whl_out[i] = limit_abs(whl_out[i], 10000.0f);
            if(fabsf(w_ref) < 0.05f) whl_out[i] = 0.0f;
        }

        /* ================= 电流输出 ================= */
        for (int i = 0; i < 4; i++)
        {
            DjiMotor_SetCurrent(&M3508[i], (int16_t)leg_out[i]);
            DjiMotor_SetCurrent(&M2006[i], (int16_t)whl_out[i]);
        }

        DjiMotor_SendGroup(M2006, 4);
        DjiMotor_SendGroup(M3508, 4);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 5ms 定时器
    if (htim->Instance == TIM7) 
    {
        flag_motor_comm = 1; 
    }
    // 10ms 定时器
    else if (htim->Instance == TIM6)
    {
        static uint8_t send_cnt = 0;
        send_cnt++;
        if (send_cnt >= 10) // 100ms
        {
            send_cnt = 0;
            flag_100ms_comm = 1; 
        }
    }
}

void DJI_OnData(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    uint8_t motor_id;

    // 过滤非法数据
    if ((header == NULL) || (data == NULL)
        || (header->IDE != CAN_ID_STD) || (header->RTR != CAN_RTR_DATA)
        || (header->DLC != DJI_MOTOR_DATA_LENGTH)
        || (header->StdId < DJI_MOTOR_FEEDBACK_ID_BASE)
        || (header->StdId > (DJI_MOTOR_FEEDBACK_ID_BASE + DJI_MOTOR_MAX_ID - 1U)))
    {
        return;
    }

    // 计算电机 ID (1~8)
    motor_id = (uint8_t)(header->StdId - DJI_MOTOR_FEEDBACK_ID_BASE + 1U);
    
    if (motor_id <= 4U)
    {
        DjiMotor_ParseFeedback(&M2006[motor_id - 1U], data);
    }
    else
    {
        DjiMotor_ParseFeedback(&M3508[motor_id - 5U], data);
    }
}

float pid_use(pid_t *pid, float kp, float ki, float kd, float target, float feedback)
{
    if (pid == NULL) return 0.0f;

    if ((pid->kp != kp) || (pid->ki != ki) || (pid->kd != kd))
    {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
        
        PID_Reset(pid); 
    }
    return PID_Calc(pid, target, feedback);
}

static void Calc_Height_Sync_Comp(dji_motor_t *motors[4], float target_height[4], float comp_out[4], float k_sync)
{
    float err_sum = 0.0f;
    float err_avg = 0.0f;
    float err[4] = {0}; // 保存每条腿的误差

    // 1. 计算每条腿的 当前位置 - 目标位置 的差值，并累加
    for (int i = 0; i < 4; i++)
    {
        err[i] = target_height[i] - motors[i]->total_encoder;
        err_sum += err[i];
    }
    
    // 2. 求这 4 个差值的平均值
    err_avg = err_sum / 4.0f;

    // 3. 计算每条腿的补偿速度（加入死区过滤）
    for (int i = 0; i < 4; i++)
    {
        float sync_err = err[i] - err_avg; 
    
            comp_out[i] = k_sync * sync_err; 
    }
}
