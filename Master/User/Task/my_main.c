/* ????????????????????????????????????????????????*/
#include "my_main.h"
#include "upper.h"
#include "status_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "can_board.h"
#include "flash.h"
#include "gpio.h"
#include "STP-23L.h"
#include "usart.h"
#include "spi.h"
#include "remote.h"

volatile uint8_t g_motor_control_enable = 0U;
volatile uint8_t g_motor_power_recovery_enable = 1U;
volatile uint8_t g_rs_recovery_motion_release = 1U;
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

void Error_Handler_Set(Error_Code code, const char *file, uint32_t line)
{
    g_error_code = code;
    g_error_handler_file = file;
    g_error_handler_line = line;
    g_error_handler_return_address = (uint32_t)__builtin_return_address(0);
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

/* STP-23L ???????????*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart9)
    {
        Remote_RxByte();
        return;
    }
    STP23L_HandleUartRxComplete(huart);
}

void My_Init(void)
{
    /* PC4/PC5 ?????????Flash ??? 8 ??SPI ????*/
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    /* Flash commands use 8-bit SPI frames. */
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
      ERROR_HANDLER(ERROR_CODE_W25Q128_INIT);
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    if (BspCan_Init() != HAL_OK)
    {
        ERROR_HANDLER(ERROR_CODE_CAN_INIT);
    }

    Upper_Init(&upper_state);
    Arm_Init();

    if (Flash_ReadId() == 0U)
    {
        ERROR_HANDLER(ERROR_CODE_W25Q128_INIT);
    }

    /* ???????????????????????????????*/
    (void)STP23L_Init(&huart2);
    (void)BspUart_RegisterRxCallback(BSP_UART_SMALL_BOARD, Remote_RxBytes);
    Remote_Init();

}

void My_RtosInit(void)
{
    /* ??????????????? CAN ????????????????*/
    if (!BspCan_RxQueueInit())
    {
        ERROR_HANDLER(ERROR_CODE_CAN_INIT);
    }
    can_rx_task_handle = osThreadNew(BspCan_RxTask, NULL, &can_rx_task_attributes);
    if (can_rx_task_handle == NULL)
    {
        ERROR_HANDLER(ERROR_CODE_CAN_INIT);
    }
    if (!Upper_TxQueueInit())
    {
        ERROR_HANDLER(ERROR_CODE_CAN_INIT);
    }
    {
        const osThreadAttr_t attributes = {
            .name = "Arm_Can_Tx_Task",
            .stack_size = 2048U,
            .priority = osPriorityNormal,
        };
        if (osThreadNew(Upper_TxTask, NULL, &attributes) == NULL)
        {
            ERROR_HANDLER(ERROR_CODE_CAN_INIT);
        }
    }
    uart_task_handle = osThreadNew(BspUart_Task, NULL, &uart_task_attributes);
    if (uart_task_handle == NULL)
    {
        ERROR_HANDLER(ERROR_CODE_USART_INIT);
    }
}
