/*
 * @file my_main.h
 * @brief my_main模块的用户接口与实现。
 */

#ifndef MY_MAIN_H
#define MY_MAIN_H

/* 功能：初始化My_Init对应的外设、参数和运行状态。 */
void My_Init(void);

/* 功能：执行My_Loop对应的周期任务并维护运行状态。 */
void My_Loop(void);
#endif
