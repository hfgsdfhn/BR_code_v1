#include "chassis_kinematics.h"

#include <stddef.h>

//1 - rr 03
//2 + lf 04
//3 + lr 01
//0 _ rf 02
/* 底盘参数 */
#define WHEEL_RADIUS        0.0775f   // 轮子半径，m
#define CHASSIS_LENGTH      0.21f      // 前后方向半轴距，m
#define CHASSIS_WIDTH       0.21f      // 左右方向半轴距，m
#define AK60_POLE_PAIRS    21.0f
#define TWO_PI             6.283185307f
#define SQRT_2              1.41421356f

/* 电机正向滚动符号，顺序：RF、RR、LF、LR */
static const float motor_direction[4] =
{
    -1.0f,
    -1.0f,
     1.0f,
     1.0f
};

/**
 * @brief 底盘逆运动学
 * @param target 底盘目标速度
 * @param motor_speed 四个电机目标角速度，rad/s
 */
void Chassis_InverseKinematics(const chassis_speed_t *target,
                               float *motor_speed)
{
    float l = CHASSIS_LENGTH + CHASSIS_WIDTH;
    float k = (60.0f * AK60_POLE_PAIRS * SQRT_2) / (TWO_PI * WHEEL_RADIUS);

    motor_speed[0] = motor_direction[0] * k *
                     (target->vx + target->vy + l * target->vw);

    motor_speed[1] = motor_direction[1] * k *
                     (target->vx - target->vy + l * target->vw);

    motor_speed[2] = motor_direction[2] * k *
                     (target->vx - target->vy - l * target->vw);

    motor_speed[3] = motor_direction[3] * k *
                     (target->vx + target->vy - l * target->vw);
}

/**
 * @brief 底盘正运动学
 * @param motor_speed 四个电机实际角速度，rad/s
 * @param actual 底盘实际速度
 */
void Chassis_ForwardKinematics(const float motor_speed[4],
                                chassis_speed_t *actual)
{
    float l = CHASSIS_LENGTH + CHASSIS_WIDTH;
    float k = (60.0f * AK60_POLE_PAIRS * SQRT_2) / (TWO_PI * WHEEL_RADIUS);

    float w0 = motor_direction[0] * motor_speed[0];
    float w1 = motor_direction[1] * motor_speed[1];
    float w2 = motor_direction[2] * motor_speed[2];
    float w3 = motor_direction[3] * motor_speed[3];

    actual->vx = (w0 + w1 + w2 + w3) / (4.0f * k);

    actual->vy = (w0 - w1 - w2 + w3) / (4.0f * k);

    actual->vw = (w0 + w1 - w2 - w3) / (4.0f * ( k /SQRT_2) * l);
}

/**
 * @brief 底盘力学逆解算
 * @param target 底盘目标力和扭矩
 * @param motor_torque 四个电机目标转矩，N·m
 */
void Chassis_ForceInverseKinematics(const chassis_force_t *target,
                                    float motor_torque[4])
{
    float l = CHASSIS_LENGTH + CHASSIS_WIDTH;
    float k = (60.0f * AK60_POLE_PAIRS * SQRT_2) / (TWO_PI * WHEEL_RADIUS);

    if ((target == NULL) || (motor_torque == NULL))
        return;

    motor_torque[0] = motor_direction[0] * k *
                     (target->fx + target->fy + target->mz / l);

    motor_torque[1] = motor_direction[1] * k *
                     (target->fx - target->fy + target->mz / l);

    motor_torque[2] = motor_direction[2] * k *
                     (target->fx - target->fy - target->mz / l);

    motor_torque[3] = motor_direction[3] * k *
                     (target->fx + target->fy - target->mz / l);
}
//1 - rr 03
//2 + lf 04
//3 + lr 01
//0 _ rf 02
