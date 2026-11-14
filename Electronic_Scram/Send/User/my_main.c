/*
 * @file my_main.c
 * @brief my_main模块的用户接口与实现。
 */

#include "main.h"
#include "my_main.h"
#include "remote_link.h"

/* 功能：初始化RemoteLink_UserGpioInit对应的外设、参数和运行状态。 */
static void RemoteLink_UserGpioInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* 功能：初始化My_Init对应的外设、参数和运行状态。 */
void My_Init(void)
{
    RemoteLink_UserGpioInit();
    RemoteLink_Init();
}

/* 功能：执行My_Loop对应的周期任务并维护运行状态。 */
void My_Loop(void)
{
    RemoteLink_PollSwitches();
    RemoteLink_Process();
    RemoteLink_ForwardRawFrame();
    HAL_Delay(1U);
}
