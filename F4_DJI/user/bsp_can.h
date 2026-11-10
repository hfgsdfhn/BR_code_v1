#ifndef BSP_CAN_H
#define BSP_CAN_H

#define use_can  0      /* 1:fdcan(H7)  0:can(F4) */

#include <stdint.h>
#include <stdbool.h>

#if use_can
#include "fdcan.h"
typedef FDCAN_HandleTypeDef BSP_CAN_Handle;
#else
#include "can.h"
typedef CAN_HandleTypeDef   BSP_CAN_Handle;
#endif

typedef struct {
    uint32_t id;
    uint8_t  ext;
    uint8_t  dlc;
    uint8_t  data[8];
} BSP_CAN_Frame_t;

typedef struct bsp_can bsp_can_t;

typedef enum
{
    std = 0,    //标准帧
    ext,        //扩展帧
}can_type_t;

struct bsp_can
{
    BSP_CAN_Handle *handle;
    can_type_t can_type;
    uint8_t         can_id;
    void (*on_rx)(bsp_can_t *self, BSP_CAN_Frame_t *frame);
};

void BSP_CAN_Init(bsp_can_t *self, can_type_t can_type);
bool BSP_CAN_Send(bsp_can_t *self, BSP_CAN_Frame_t *frame);
void BSP_CAN_Bind(bsp_can_t *can);      //init后必须调用它注册需要的can
#endif