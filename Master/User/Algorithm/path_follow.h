
#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <stdbool.h>
#include "chassis.h"
#include <stdint.h>

typedef struct
{
    const void *path;
    uint16_t path_size;
    uint16_t nearest_index;
    float lookahead_mm;
    float normal_gain;
    float normal_speed_limit;
    float yaw_gain;
    float yaw_speed_limit;
    float finish_distance_mm;
    bool finished;
} path_follow_t;

bool PathFollow_Init(path_follow_t *follow,     // 路径跟踪结构体
                     const void *path,          // 路径点数组
                     uint16_t path_size,        // 路径点数量
                     float lookahead_mm,        // 前视距离（mm）
                     float normal_gain,         // 法向 P 增益
                     float normal_speed_limit,  // 法向速度限制（m/s）
                     float finish_distance_mm);  // 结束距离（mm）

void PathFollow_Reset(path_follow_t *follow);

bool PathFollow_Calc(path_follow_t *follow,
                     float x_m,
                     float y_m,
                     chassis_speed_t *speed);

extern path_follow_t path_to_xiepo;


#endif
