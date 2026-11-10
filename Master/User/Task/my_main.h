/* 搴旂敤鎬诲垵濮嬪寲涓庡叕鍏辨帴鍙ｃ€?*/
#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "stm32h7xx_hal.h"

typedef enum
{
    ERROR_CODE_NONE = 0,         /* 鏃犻敊璇€?*/
    ERROR_CODE_SYSTEM_CLOCK,     /* 绯荤粺鏃堕挓鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_PERIPHERAL_CLOCK, /* 澶栬鏃堕挓鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_SPI6_INIT,        /* SPI6 鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_FDCAN1_INIT,      /* FDCAN1 鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_FDCAN2_INIT,      /* FDCAN2 鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_FDCAN3_INIT,      /* FDCAN3 鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_M2006_INIT,       /* M2006 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_M3508_INIT,       /* M3508 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_RS00_INIT,        /* RS00 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_RS05_INIT,        /* RS05 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_AK60_INIT,        /* AK60 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_J4310_INIT,       /* J4310 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_RS02_INIT,        /* RS02 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_VESC_INIT,        /* VESC 鐢垫満鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_CAN_INIT,         /* CAN 鏈嶅姟鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_USART_INIT,       /* USART 鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_W25Q128_INIT,     /* W25Q128 闂瓨 鍒濆鍖栧け璐ャ€?*/
    ERROR_CODE_HAL_HANDLER,      /* HAL 閿欒澶勭悊鍏ュ彛銆?*/
    ERROR_CODE_ASSERT,
    ERROR_CODE_HARDFAULT,
    ERROR_CODE_MEMMANAGE,
    ERROR_CODE_BUSFAULT,
    ERROR_CODE_USAGEFAULT
} Error_Code;

extern volatile Error_Code g_error_code;
extern volatile const char *g_error_handler_file;
extern volatile uint32_t g_error_handler_line;
extern volatile uint32_t g_error_handler_return_address;
extern volatile uint32_t g_error_handler_ipsr;
extern volatile uint32_t g_error_handler_cfsr;
extern volatile uint32_t g_error_handler_hfsr;
extern volatile uint32_t g_error_handler_bfar;
extern volatile uint32_t g_error_handler_mmfar;

void Error_FaultCapture(Error_Code code, uint32_t *stack);
/* 鍑芥暟 Error_Handler_Set锛氭墽琛岃鎺ュ彛瀵瑰簲鐨勬帶鍒舵垨鏁版嵁澶勭悊娴佺▼銆?*/
void Error_Handler_Set(Error_Code code /* 閿欒绫诲瀷銆?*/, const char *file /* 浜х敓閿欒鐨勬簮鏂囦欢鍚嶃€?*/,
                       uint32_t line /* 浜х敓閿欒鐨勬簮浠ｇ爜琛屽彿銆?*/);
#define ERROR_HANDLER(code) Error_Handler_Set((code), __FILE__, (uint32_t)__LINE__) /* 璁板綍閿欒鐮併€佹枃浠跺拰琛屽彿銆?*/

/* 鍔熻兘锛氭墽琛岀敤鎴风‖浠躲€佹帶鍒跺璞″拰閫氫俊閾捐矾鍒濆鍖栵紱鍒濆鍖栧簲鐢ㄥ眰銆侀€氫俊鎺ュ彛銆侀棯瀛?鍜屽叏閮ㄧ數鏈烘帶鍒跺璞°€?*/
void My_Init(void);
/* 鐜版湁鏉跨骇鏄犲皠锛欴M-IMU 鎺ュ湪 USART1 涓婏紱鍙寜闇€瑕嗙洊鍏惰妭鐐?ID銆?*/
#ifndef MY_IMU_DEVICE_ID
#define MY_IMU_DEVICE_ID 0U

#endif
#define MY_IMU_TIMEOUT_MS 200U

void My_OnImuRx(const uint8_t *data, uint16_t size);
extern volatile float g_whole_yaw_deg;
extern volatile uint8_t g_whole_yaw_valid;
/* 鏈繛鎺?IMU 杩涜鍙版灦娴嬭瘯鏃讹紝鍙湪 鍙橀噺瑙傚療绐楀彛 涓樉寮忚鐩栨璁剧疆銆?*/
extern volatile uint8_t g_whole_yaw_manual;
/* 鍦?osKernelInitialize() 涔嬪悗銆佽皟搴﹀櫒鍚姩涔嬪墠璋冪敤銆?*/
void My_RtosInit(void);

extern volatile uint8_t g_motor_control_enable;
/* 0=鍏抽棴鐢垫満鎺夌數鍥為浂锛?=鐢垫満鎺夌數閲嶄笂鐢靛悗鍥炲埌闆剁偣骞朵繚鎸併€?*/
extern volatile uint8_t g_motor_power_recovery_enable;
extern volatile uint8_t g_rs_recovery_motion_release;

#endif

