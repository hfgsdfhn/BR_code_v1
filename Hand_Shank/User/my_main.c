/*
 * @file my_main.c
 * @brief my_main模块的用户接口与实现。
 */

#include "my_main.h"
#include "remote.h"


/* 功能：执行 My_Init 函数对应的模块操作。 */
void My_Init(void)
{
    Remote_Init();
}


/* 功能：执行 My_Loop 函数对应的模块操作。 */
void My_Loop(void)
{
    Remote_Loop();
}
