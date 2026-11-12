/*
 * @file my_main.c
 * @brief my_main模块的用户接口与实现。
 */

#include "my_main.h"
#include "remote.h"


/* 功能：初始化模块并准备外设资源。 */
void My_Init(void)
{
    Remote_Init();
}


/* 功能：执行 My_Loop 模块操作并维护相关状态。 */
void My_Loop(void)
{
    Remote_Loop();
}
