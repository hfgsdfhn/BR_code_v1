
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
    uint16_t nearest_index;
    float lookahead_mm;
    float normal_gain;
    float normal_speed_limit;
    float yaw_gain;
    float yaw_speed_limit;
    float finish_distance_mm;
    bool finished;
} path_follow_t;

bool PathFollow_Init(path_follow_t *follow,
                     const void *path,
                     uint16_t path_size,
                     float lookahead_mm,
                     float normal_gain,
                     float normal_speed_limit,
                     float yaw_gain,
                     float yaw_speed_limit,
                     float finish_distance_mm);

void PathFollow_Reset(path_follow_t *follow);

bool PathFollow_Calc(path_follow_t *follow,
                     float x_m,
                     float y_m,
                     float yaw_rad,
                     path_follow_speed_t *speed);

#endif
