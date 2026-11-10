#include "my_main.h"
#include "remote.h"

/* 初始化 F407 遥控器业务；UART6、ADC1 和 GPIO 已由 main.c 完成初始化。 */
void My_Init(void)
{
    Remote_Init();
}

/* 周期调用 remote.c，采集摇杆和按键状态，并通过 UART6 DMA 发送遥控帧。 */
void My_Loop(void)
{
    Remote_Loop();
}
