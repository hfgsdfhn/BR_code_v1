/*
 * @file my_main.c
 * @brief my_main?????????????????
 */

#include "my_main.h"
#include "upper.h"
#include "arm_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "scram.h"
#include "flash.h"
#include "usart.h"
#include "spi.h"
#include "fdcan.h"
#include "remote.h"
#include "suction_cup.h"
#include "../Mechanism/Upper/arm_config.h"
#include "../Mechanism/Upper/cup_config.h"

m2006_t m2006_larm;
m2006_t m2006_ucup[2];
m2006_t m2006_dcup[2];
m3508_t m3508_dcup[1];
rs00_t rs00_larm[2], rs00_rarm[2];
rs05_t rs05_larm, rs05_rarm;
j4310_t j4310_larm, j4310_rarm;
rs01_t rs01_rarm;


volatile Error_Code g_error_code = ERROR_CODE_NONE;
volatile const char *g_error_handler_file = NULL;
volatile uint32_t g_error_handler_line = 0U;
volatile uint32_t g_error_handler_return_address = 0U;
volatile uint32_t g_error_handler_ipsr = 0U;
volatile uint32_t g_error_handler_cfsr = 0U;
volatile uint32_t g_error_handler_hfsr = 0U;
volatile uint32_t g_error_handler_bfar = 0U;
volatile uint32_t g_error_handler_mmfar = 0U;

static osThreadId_t can_rx_task_handle;
static osThreadId_t uart_task_handle;
static const osThreadAttr_t uart_task_attributes = {
    .name = "Uart_Task",
    .stack_size = 2048U,
    .priority = osPriorityNormal,
};
static const osThreadAttr_t can_rx_task_attributes = {
    .name = "Can_Rx_Task",
    .stack_size = 2048U,
    .priority = osPriorityNormal,
};

/* ?????????? */
void Error_FaultCapture(Error_Code code, uint32_t *stack)
{
    g_error_code = code;
    g_error_handler_file = "CPU Fault";
    g_error_handler_line = 0U;
    g_error_handler_return_address = (stack != NULL) ? stack[6] : 0U;
    g_error_handler_ipsr = __get_IPSR();
    g_error_handler_cfsr = SCB->CFSR;
    g_error_handler_hfsr = SCB->HFSR;
    g_error_handler_bfar = SCB->BFAR;
    g_error_handler_mmfar = SCB->MMFAR;
    __disable_irq();
    while (1)
    {
    }
}

/* ?????????? */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

<<<<<<< HEAD
 
/* 功能：执行 My_Init 函数对应的模块操作。 */
=======

/* ?????????? */
>>>>>>> c1f3984123493276c22fa7685c0e10887a5564d8
void My_Init(void)
{
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;

    (void)HAL_SPI_Init(&hspi1);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    if (BspCan_Init() != HAL_OK)
    {
        g_error_code = ERROR_CODE_CAN_INIT;
    }

    M2006_Init(&m2006_dcup[0], &hfdcan2, 2U, -1);
    M2006_Init(&m2006_dcup[1], &hfdcan2, 3U, -1);
    M3508_Init(&m3508_dcup[0], &hfdcan2, 5U, 1);

    M2006_Init(&m2006_ucup[0], &hfdcan3, 2U, 1);
    M2006_Init(&m2006_ucup[1], &hfdcan3, 3U, 1);

    M2006_Init(&m2006_larm, &hfdcan2, 1U, 1);
    RS00_Init(&rs00_larm[0], &hfdcan2, 2U, 1);
    RS00_Init(&rs00_larm[1], &hfdcan2, 3U, 1);
    J4310_Init(&j4310_larm, &hfdcan2, 4U, 1);
    RS05_Init(&rs05_larm, &hfdcan2, 5U, 1);

    RS01_Init(&rs01_rarm, &hfdcan3, 1U, -1);
    RS00_Init(&rs00_rarm[0], &hfdcan3, 2U, -1);
    RS00_Init(&rs00_rarm[1], &hfdcan3, 3U, -1);
    J4310_Init(&j4310_rarm, &hfdcan3, 4U, -1);
    RS05_Init(&rs05_rarm, &hfdcan3, 5U, -1);
    configure_motor_gains();
    Arm_TaskInit();
    Upper_BindMotors(&upper_state);
    (void)BspUart_RegisterRxCallback(BSP_UART_REMOTE, Remote_Receive);
    SuctionCup_Init();
    Scram_Init();
    Remote_Init();
}

/* ?????????? */
void My_RtosInit(void)
{

    (void)BspCan_RxQueueInit();
    can_rx_task_handle = osThreadNew(BspCan_RxTask, NULL, &can_rx_task_attributes);
    (void)Arm_TxQueueInit();
    {
        const osThreadAttr_t attributes = {
            .name = "Arm_Can_Tx_Task",
            .stack_size = 2048U,
            .priority = osPriorityNormal,
        };
        (void)osThreadNew(Arm_TxTask, NULL, &attributes);
    }
    uart_task_handle = osThreadNew(BspUart_Task, NULL, &uart_task_attributes);
}
