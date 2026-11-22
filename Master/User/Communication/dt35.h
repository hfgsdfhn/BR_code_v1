#ifndef DT35_H
#define DT35_H

#include <stdint.h>

typedef struct
{
    float dt35_x;
    float dt35_y;
} dt35_t;

void DT35_RxBytes(const uint8_t *data, uint16_t size);
uint8_t DT35_GetLatest(dt35_t *data);

#endif