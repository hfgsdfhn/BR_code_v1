/*
 * @file path_follow.h
 * @brief path_follow模块的用户接口与实现。
 */

#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float vx;
    float vy;
    float vw;
} path_follow_speed_t;

typedef struct
{
    const void *path;
    uint16_t path_size;
    uint16_t nearest_index; /* 元素索引。 */
    float lookahead_mm;
    float normal_gain;
    float normal_speed_limit;
    float yaw_gain;
    float yaw_speed_limit;
    float finish_distance_mm;
    bool finished;
} path_follow_t;

/* 功能：初始化PathFollow_Init对应的外设、参数和运行状态。 */
bool PathFollow_Init(path_follow_t *follow,
                     const void *path,
                     uint16_t path_size,
                     float lookahead_mm,
                     float normal_gain,
                     float normal_speed_limit,
                     float yaw_gain,
                     float yaw_speed_limit,
                     float finish_distance_mm);

/* 功能：重置或释放PathFollow_Reset对应的内部状态和资源。 */
void PathFollow_Reset(path_follow_t *follow);

/* 功能：计算PathFollow_Calc对应的控制量或规划结果。 */
bool PathFollow_Calc(path_follow_t *follow,
                     float x_m,       /* 当前世界坐标 x，单位 m。 */
                     float y_m,       /* 当前世界坐标 y，单位 m。 */
                     float yaw_rad,
                     path_follow_speed_t *speed);

#endif
