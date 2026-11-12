# LoRa 接收板到主控 UART7

链路：LoRa TX → Receive PB11/USART3_RX → 512 字节 DMA 环形缓冲 → 1024 字节发送队列 → PA9/USART1_TX → Master PE7/UART7_RX → 256 字节 DMA 环形缓冲 → scram.c 的 1024 字节原始接收队列。

- Receive PA9 接 Master PE7，双方 GND 共地；保留反向透传时 Master PE8/UART7_TX 接 Receive PA10/USART1_RX。
- 这三路串口均为 115200、8N1、无硬件流控。
- 字节值和顺序不变，不加帧头、不附加换行、不解码协议。DMA 分段不代表应用报文边界。
- 接收 DMA 使用 CIRCULAR，发送 DMA 使用 NORMAL，发送软件队列负责回绕。
- 主控 My_Init() 调用 Scram_Init() 注册 BSP_UART_SCRAM 回调；随后 BspUart_Task 启动 UART7 DMA。HT/TC/IDLE 中断记录接收进度，任务轮询复制新字节，串口错误由任务重新启动接收。
- 主控 bsp_uart.o 的数据由 Master/MDK-ARM/uart_dma.sct 放入 AXI SRAM，接收缓冲按 32 字节对齐并维护 DCache。重新生成 Keil 工程时应保留该 scatter 文件配置。

上层任务应定期读取；读取长度可能小于请求长度，也可能跨多个报文，需要上层自行组帧：

```c
#include "scram.h"

uint8_t data[128];
uint16_t size = Scram_Read(data, sizeof(data));

```

Scram_Read() 为非阻塞接口，建议使用单一消费者。Scram_GetStats() 提供收到字节数、软件队列丢弃字节数、最后接收时间和未读字节数；DMA 缓冲覆盖次数可在调试器观察 uart_list[BSP_UART_SCRAM].rx_overruns。

队列满时保留旧的未读数据，丢弃无法入队的新字节并计数；DMA 已覆盖的数据整段丢弃。无流控且缓冲有限，应用不及时读取、UART 任务长期得不到调度或中断被屏蔽超过一圈接收时间时，不能保证无丢失。此模块只收取数据，不根据载荷执行急停或失联动作。

验证：接收板 Keil 构建通过（0 错误、0 警告）；tests/scram_dma 使用实际 bsp_uart.c、scram.c 做主机模拟，覆盖原始字节、重复事件、整圈接收、回绕、溢出、错误恢复及 32 位计数回绕。模拟不替代硬件与 LoRa 联调。

主控通信源码编译通过，但完整链接仍被工作区原有的 can_board.c 删除阻塞：该文件包含 Upper_Init、Arm_Init、Upper_TxTask 等机械臂实现，当前缺少这些定义。未恢复该项既有删除。构建日志见 Master/MDK-ARM/scram_build.log。
