#include "main.h"
#include "my_main.h"
#include "remote_link.h"

void My_Init(void)
{
    RemoteLink_Init();
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,1);
}

void My_Loop(void)
{
    RemoteLink_Process();
    HAL_Delay(1U);
}
