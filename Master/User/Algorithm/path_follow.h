/*
 * @file path_follow.h
 * @brief path_follow模块的用户接口与实现。
 */

#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

/* 模块实现细节。 */
typedef struct
{
    float vx; /* 成员变量。 */
    float vy; /* 成员变量。 */
    float vw; /* 成员变量。 */
} path_follow_speed_t;

typedef struct
{
    const void *path; /* 成员变量。 */
    uint16_t path_size; /* 数据长度或数量。 */
    uint16_t nearest_index; /* 元素索引。 */
    float lookahead_mm; /* 成员变量。 */
    float normal_gain; /* 成员变量。 */
    float normal_speed_limit; /* 成员变量。 */
    float yaw_gain; /* 成员变量。 */
    float yaw_speed_limit; /* 成员变量。 */
    float finish_distance_mm; /* 成员变量。 */
    bool finished; /* 成员变量。 */
} path_follow_t;

/* 功能：初始化模块并准备外设资源。 */
bool PathFollow_Init(path_follow_t *follow, /* 成员变量。 */
                     const void *path, /* 成员变量。 */
                     uint16_t path_size, /* 数据长度或数量。 */
                     float lookahead_mm, /* 成员变量。 */
                     float normal_gain, /* 成员变量。 */
                     float normal_speed_limit, /* 成员变量。 */
                     float yaw_gain, /* 成员变量。 */
                     float yaw_speed_limit, /* 成员变量。 */
                     float finish_distance_mm);

/* 功能：执行 PathFollow_Reset 模块操作并维护相关状态。 */
void PathFollow_Reset(path_follow_t *follow);

/* 功能：执行 PathFollow_Calc 模块操作并维护相关状态。 */
bool PathFollow_Calc(path_follow_t *follow, /* 成员变量。 */
                     float x_m,       /* 当前世界坐标 x，单位 m。 */
                     float y_m,       /* 当前世界坐标 y，单位 m。 */
                     float yaw_rad,   /* 模块实现细节。 */
                     path_follow_speed_t *speed);

#endif
