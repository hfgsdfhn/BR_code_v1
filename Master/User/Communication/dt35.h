#ifndef DT35_LINK_H
#define DT35_LINK_H

#include <stdint.h>

typedef struct
{
    float left_distance;   // 左侧 DT35，单位 cm
    float front_distance;  // 前侧 DT35，单位 cm
} dt35_data_t;

extern dt35_data_t dt35_data;

void DT35_RxBytes(const uint8_t *data, uint16_t size);

#endif