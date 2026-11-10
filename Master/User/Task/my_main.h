/* 主控应用初始化、故障记录及公共状态接口。 */
#ifndef MY_MAIN_H
#define MY_MAIN_H

#include "stm32h7xx_hal.h"

typedef enum
{
    ERROR_CODE_NONE = 0,         /* 无错误。 */
    ERROR_CODE_SYSTEM_CLOCK,     /* 系统时钟初始化失败。 */
    ERROR_CODE_PERIPHERAL_CLOCK, /* 外设时钟初始化失败。 */
    ERROR_CODE_SPI6_INIT,        /* SPI6 初始化失败。 */
    ERROR_CODE_FDCAN1_INIT,      /* FDCAN1 初始化失败。 */
    ERROR_CODE_FDCAN2_INIT,      /* FDCAN2 初始化失败。 */
    ERROR_CODE_FDCAN3_INIT,      /* FDCAN3 初始化失败。 */
    ERROR_CODE_M2006_INIT,       /* M2006 电机初始化失败。 */
    ERROR_CODE_M3508_INIT,       /* M3508 电机初始化失败。 */
    ERROR_CODE_RS00_INIT,        /* RS00 电机初始化失败。 */
    ERROR_CODE_RS05_INIT,        /* RS05 电机初始化失败。 */
    ERROR_CODE_AK60_INIT,        /* AK60 电机初始化失败。 */
    ERROR_CODE_J4310_INIT,       /* J4310 电机初始化失败。 */
    ERROR_CODE_RS02_INIT,        /* RS02 电机初始化失败。 */
    ERROR_CODE_VESC_INIT,        /* VESC 电机初始化失败。 */
    ERROR_CODE_CAN_INIT,         /* CAN 通信服务初始化失败。 */
    ERROR_CODE_USART_INIT,       /* 串口通信服务初始化失败。 */
    ERROR_CODE_W25Q128_INIT,     /* W25Q128 闪存初始化失败。 */
    ERROR_CODE_HAL_HANDLER,      /* 进入 HAL 通用错误处理入口。 */
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
/* 记录错误位置、返回地址和故障寄存器，随后关闭中断并停留在死循环中。 */
void Error_Handler_Set(Error_Code code /* 错误类型。 */, const char *file /* 发生错误的源文件名。 */,
                       uint32_t line /* 发生错误的源代码行号。 */);
#define ERROR_HANDLER(code) Error_Handler_Set((code), __FILE__, (uint32_t)__LINE__) /* 自动记录调用处的文件名和行号。 */

/* 初始化主控 GPIO、Flash、CAN、机械臂、测距传感器和遥控接收模块。 */
void My_Init(void);
/* IMU 设备标识的默认值；可通过预定义宏覆盖。 */
#ifndef MY_IMU_DEVICE_ID
#define MY_IMU_DEVICE_ID 0U

#endif
#define MY_IMU_TIMEOUT_MS 200U

void My_OnImuRx(const uint8_t *data, uint16_t size);
extern volatile float g_whole_yaw_deg;
extern volatile uint8_t g_whole_yaw_valid;
/* 整机偏航角手动输入标志，供未连接 IMU 时的台架调试使用。 */
extern volatile uint8_t g_whole_yaw_manual;
/* 在 osKernelInitialize() 之后、调度器启动之前创建通信队列和任务。 */
void My_RtosInit(void);

extern volatile uint8_t g_motor_control_enable;
/* 电机重新上电后的恢复运动开关：0 为关闭，1 为启用。 */
extern volatile uint8_t g_motor_power_recovery_enable;
extern volatile uint8_t g_rs_recovery_motion_release;

#endif

