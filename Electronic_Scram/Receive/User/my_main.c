/*
 * @file my_main.c
 * @brief my_main妯″潡鐨勭敤鎴锋帴鍙ｄ笌瀹炵幇銆? */

#include "main.h"
#include "my_main.h"
#include "remote_link.h"

/* 鍔熻兘锛氬垵濮嬪寲My_Init瀵瑰簲鐨勫璁俱€佸弬鏁板拰杩愯鐘舵€併€?*/
void My_Init(void)
{
    /* PA15/PB3..PB9 initial level is set by MX_GPIO_Init() in gpio.c */
    RemoteLink_Init();
}

/* 鍔熻兘锛氭墽琛孧y_Loop瀵瑰簲鐨勫懆鏈熶换鍔″苟缁存姢杩愯鐘舵€併€?*/
void My_Loop(void)
{
    RemoteLink_Process();

}

