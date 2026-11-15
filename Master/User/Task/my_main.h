/*
 * @file my_main.h
 * @brief my_main模块的用户接口与实现。
 */

#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "stm32h7xx_hal.h"

typedef enum
{
    ERROR_CODE_NONE = 0,
    ERROR_CODE_SYSTEM_CLOCK,
    ERROR_CODE_PERIPHERAL_CLOCK,
    ERROR_CODE_SPI6_INIT,
    ERROR_CODE_FDCAN1_INIT,
    ERROR_CODE_FDCAN2_INIT,
    ERROR_CODE_FDCAN3_INIT,
    ERROR_CODE_M2006_INIT,
    ERROR_CODE_M3508_INIT,
    ERROR_CODE_RS00_INIT,
    ERROR_CODE_RS05_INIT,
    ERROR_CODE_AK60_INIT,
    ERROR_CODE_J4310_INIT,
    ERROR_CODE_RS01_INIT,
    ERROR_CODE_VESC_INIT,
    ERROR_CODE_CAN_INIT,
    ERROR_CODE_USART_INIT,
    ERROR_CODE_W25Q128_INIT,
    ERROR_CODE_HAL_HANDLER, /* 对应外设句柄。 */
    ERROR_CODE_ASSERT,
    ERROR_CODE_HARDFAULT,
    ERROR_CODE_MEMMANAGE,
    ERROR_CODE_BUSFAULT,
    ERROR_CODE_USAGEFAULT
} Error_Code;

extern volatile Error_Code g_error_code;
extern volatile const char *g_error_handler_file; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_line; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_return_address; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_ipsr; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_cfsr; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_hfsr; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_bfar; /* 对应外设句柄。 */
extern volatile uint32_t g_error_handler_mmfar; /* 对应外设句柄。 */

/* 功能：处理Error_FaultCapture对应的输入并更新相关状态。 */
void Error_FaultCapture(Error_Code code, uint32_t *stack);

/* 功能：初始化My_Init对应的外设、参数和运行状态。 */
void My_Init(void);

#ifndef MY_IMU_DEVICE_ID
#define MY_IMU_DEVICE_ID 0U

#endif
#define MY_IMU_TIMEOUT_MS 200U

/* 功能：解析 IMU 接收数据并更新全局航向角。 */
void My_OnImuRx(const uint8_t *data, uint16_t size);
extern volatile float g_whole_yaw_deg;
extern volatile uint8_t g_whole_yaw_valid;

extern volatile uint8_t g_whole_yaw_manual;

/* 功能：初始化My_RtosInit对应的外设、参数和运行状态。 */
void My_RtosInit(void);

extern volatile uint8_t g_rs_recovery_motion_release;

#endif
