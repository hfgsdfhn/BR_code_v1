
#include "path_follow.h"
#include <math.h>

typedef struct
{
    float x;
    float y;
    float theta;
    float curvature;
    float s;
    float velocity;
} path_point_t;

static float PathFollow_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PathFollow_Clamp(float value, float limit)
{
    limit = PathFollow_Abs(limit);
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static float PathFollow_WrapPi(float angle)
{
    while (angle >= 3.1415926f)
    {
        angle -= 6.2831852f;
    }
    while (angle < -3.1415926f)
    {
        angle += 6.2831852f;
    }
    return angle;
}

bool PathFollow_Init(path_follow_t *follow,     // 路径跟踪结构体
                     const void *path,          // 路径点数组
                     uint16_t path_size,        // 路径点数量
                     float lookahead_mm,        // 前视距离（mm）
                     float normal_gain,         // 法向 P 增益
                     float normal_speed_limit,  // 法向速度限制（m/s）
                     float finish_distance_mm)  // 结束距离（mm）
{
    if ((follow == 0) || (path == 0) || (path_size == 0U) ||
        (lookahead_mm < 0.0f) || (normal_gain < 0.0f) ||
        (normal_speed_limit < 0.0f) ||
        (finish_distance_mm < 0.0f))
    {
        return false;
    }

    follow->path = path;
    follow->path_size = path_size;
    follow->lookahead_mm = lookahead_mm;
    follow->normal_gain = normal_gain;
    follow->normal_speed_limit = normal_speed_limit;
    follow->finish_distance_mm = finish_distance_mm;

    PathFollow_Reset(follow);

    return true;
}


void PathFollow_Reset(path_follow_t *follow)
{
    if (follow != 0)
    {
        follow->nearest_index = 0U;
        follow->finished = false;
    }
}

bool PathFollow_Calc(path_follow_t *follow,
                     float x_m,
                     float y_m,
                     float yaw_rad,
                     path_follow_speed_t *speed)
{
    const path_point_t *points;
    const path_point_t *nearest;
    const path_point_t *target;
    uint16_t index;
    uint16_t target_index;
    float nearest_distance;
    float distance;
    float target_s;
    float dx;
    float dy;
    float tangent_x;
    float tangent_y;
    float normal_x;
    float normal_y;
    float normal_error;
    float normal_speed;
    float tangent_speed;

    if ((follow == 0) || (follow->path == 0) ||
        (follow->path_size == 0U) || (speed == 0))
    {
        return false;
    }

    if (follow->finished)
    {
        speed->vx = 0.0f;
        speed->vy = 0.0f;
        speed->vw = 0.0f;
        return true;
    }

    points = (const path_point_t *)follow->path;
    nearest = &points[follow->nearest_index];
    nearest_distance = 1.0e30f;

    /* 搜索距离机器人最近的路径点 */
    for (index = follow->nearest_index; index < follow->path_size; index++)
    {
        dx = x_m - points[index].x * 0.001f;
        dy = y_m - points[index].y * 0.001f;
        distance = dx * dx + dy * dy;

        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            follow->nearest_index = index;
            nearest = &points[index];
        }
    }

    /* 根据预瞄距离寻找目标点 */
    target_s = nearest->s + follow->lookahead_mm;
    target_index = follow->nearest_index;

    while ((target_index + 1U < follow->path_size) &&
           (points[target_index + 1U].s < target_s))
    {
        target_index++;
    }

    target = &points[target_index];

    /* 判断是否到达路径终点 */
    dx = x_m - points[follow->path_size - 1U].x * 0.001f;
    dy = y_m - points[follow->path_size - 1U].y * 0.001f;

    distance = dx * dx + dy * dy;

    if (distance <= follow->finish_distance_mm *
                    follow->finish_distance_mm * 0.000001f)
    {
        follow->finished = true;
        speed->vx = 0.0f;
        speed->vy = 0.0f;
        speed->vw = 0.0f;
        return true;
    }

    /* 计算路径切线方向和法线方向 */
    tangent_x = cosf(target->theta);
    tangent_y = sinf(target->theta);

    normal_x = -tangent_y;
    normal_y = tangent_x;

    /* 计算机器人相对于路径的法向误差 */
    dx = x_m - nearest->x * 0.001f;
    dy = y_m - nearest->y * 0.001f;

    normal_error = dx * normal_x + dy * normal_y;

    /* 法向纠偏速度 */
    normal_speed = PathFollow_Clamp(-follow->normal_gain * normal_error,
                                    follow->normal_speed_limit);

    /* 切线前进速度 */
    tangent_speed = 1.5f * target->velocity;

    /* 合成底盘速度 */
    speed->vx = tangent_speed * tangent_x + normal_speed * normal_x;
    speed->vy = tangent_speed * tangent_y + normal_speed * normal_y;

    /* 不进行角速度纠偏 */
    speed->vw = 0.0f;

    return true;
}

