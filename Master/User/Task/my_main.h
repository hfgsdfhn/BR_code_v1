/*
 * @file my_main.h
 * @brief my_main模块的用户接口与实现。
 */

#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "stm32h7xx_hal.h"

typedef enum
{
    ERROR_CODE_NONE = 0, /* 成员变量。 */
    ERROR_CODE_SYSTEM_CLOCK, /* 成员变量。 */
    ERROR_CODE_PERIPHERAL_CLOCK, /* 成员变量。 */
    ERROR_CODE_SPI6_INIT, /* 成员变量。 */
    ERROR_CODE_FDCAN1_INIT, /* 成员变量。 */
    ERROR_CODE_FDCAN2_INIT, /* 成员变量。 */
    ERROR_CODE_FDCAN3_INIT, /* 成员变量。 */
    ERROR_CODE_M2006_INIT, /* 成员变量。 */
    ERROR_CODE_M3508_INIT, /* 成员变量。 */
    ERROR_CODE_RS00_INIT, /* 成员变量。 */
    ERROR_CODE_RS05_INIT, /* 成员变量。 */
    ERROR_CODE_AK60_INIT, /* 成员变量。 */
    ERROR_CODE_J4310_INIT, /* 成员变量。 */
    ERROR_CODE_RS01_INIT, /* 成员变量。 */
    ERROR_CODE_VESC_INIT, /* 成员变量。 */
    ERROR_CODE_CAN_INIT, /* 成员变量。 */
    ERROR_CODE_USART_INIT, /* 成员变量。 */
    ERROR_CODE_W25Q128_INIT, /* 成员变量。 */
    ERROR_CODE_HAL_HANDLER, /* 对应外设句柄。 */
    ERROR_CODE_ASSERT, /* 成员变量。 */
    ERROR_CODE_HARDFAULT, /* 成员变量。 */
    ERROR_CODE_MEMMANAGE, /* 成员变量。 */
    ERROR_CODE_BUSFAULT, /* 成员变量。 */
    ERROR_CODE_USAGEFAULT /* 成员变量。 */
} Error_Code;

extern volatile Error_Code g_error_code; /* 成员变量。 */
extern volatile const char *g_error_handler_file; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_line; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_return_address; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_ipsr; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_cfsr; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_hfsr; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_bfar; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_mmfar; /* 对应外设句柄。 */

/* 功能：执行 Error_FaultCapture 函数对应的模块操作。 */
void Error_FaultCapture(Error_Code code, uint32_t *stack);

/* 功能：执行 My_Init 函数对应的模块操作。 */
void My_Init(void);

#ifndef MY_IMU_DEVICE_ID
#define MY_IMU_DEVICE_ID 0U

#endif
#define MY_IMU_TIMEOUT_MS 200U


/* 功能：执行 My_OnImuRx 函数对应的模块操作。 */
void My_OnImuRx(const uint8_t *data, uint16_t size);
extern volatile float g_whole_yaw_deg; /* 成员变量。 */
extern volatile uint8_t g_whole_yaw_valid; /* 状态标志。 */

extern volatile uint8_t g_whole_yaw_manual; /* 成员变量。 */

/* 功能：执行 My_RtosInit 函数对应的模块操作。 */
void My_RtosInit(void);


extern volatile uint8_t g_rs_recovery_motion_release; /* 成员变量。 */

#endif
