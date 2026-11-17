/*
 * @file my_main.c
 * @brief my_main濡€虫健閻ㄥ嫮鏁ら幋閿嬪复閸欙絼绗岀€圭偟骞囬妴? */

#include "main.h"
#include "my_main.h"
#include "remote_link.h"

/* 閸旂喕鍏橀敍姘灥婵瀵睷emoteLink_UserGpioInit鐎电懓绨查惃鍕樆鐠佷勘鈧礁寮弫鏉挎嫲鏉╂劘顢戦悩鑸碘偓浣碘偓?*/
static void RemoteLink_UserGpioInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* 閸旂喕鍏橀敍姘灥婵瀵睲y_Init鐎电懓绨查惃鍕樆鐠佷勘鈧礁寮弫鏉挎嫲鏉╂劘顢戦悩鑸碘偓浣碘偓?*/
void My_Init(void)
{
    RemoteLink_UserGpioInit();
    RemoteLink_Init();
}

/* 閸旂喕鍏橀敍姘⒔鐞涘y_Loop鐎电懓绨查惃鍕噯閺堢喍鎹㈤崝鈥宠嫙缂佸瓨濮㈡潻鎰攽閻樿埖鈧降鈧?*/
void My_Loop(void)
{
    RemoteLink_PollSwitches();
    RemoteLink_Process();
    RemoteLink_ForwardRawFrame();
}
