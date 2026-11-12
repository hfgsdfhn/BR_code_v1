# 遥控器直连主控 LoRa 链路

Hand_Shank USART6 TX（PC6）→ 遥控端 LoRa RX → 无线链路 → 主控端 LoRa TX → Master UART8 RX（PE0）。各板与本地 LoRa 模块共地。

两端 MCU 串口配置为 115200、8N1。LoRa 模块需预先配置为匹配的透明传输模式及无线参数；本次代码不发送模块配置命令。

- 遥控端：每 20 ms 采样一次，四槽环形发送队列，每槽 8 字节。DMA_NORMAL 每次发送一个槽，发送完成后释放槽；主循环启动下一槽。队列满时跳过本次采样，DMA 出错后清空待发帧并恢复。
- 主控端：UART8 RX 使用 DMA2 Stream2、DMA_CIRCULAR、256 字节缓冲区。HT/TC/IDLE 中断记录接收进度，UART 任务每 tick 检查并复制新增字节，交给 Remote_RxBytes() 跨批次解析。溢出或重启时清除遥控半帧和有效标志。
- 发送侧的“环形”指软件队列；发送 DMA 采用 NORMAL，接收 DMA 采用 CIRCULAR。当前链路为单向遥控数据发送，未增加遥控端 UART6 RX DMA。

帧格式保持 8 字节：A5 5A、摇杆一、按键组一、摇杆二、按键组二、按键组三、按键组四。主控保留 shoulder 字段名以兼容现有接口，其内容实际是第二组 ADC 摇杆的两个半字节。协议没有长度或 CRC 字段。

## 上板检查

1. 烧录前确认主控工程可完整链接；本次构建的主控链接日志为 Master/MDK-ARM/lora_build.log。当前 Upper/Arm 等未定义符号需要由相关模块补齐。
2. 检查遥控端 PC6 和主控端 PE0 的串口数据，确认每帧为 A5 5A 开头的 8 字节。
3. 观察 Remote_GetValidFrameCount() 持续增长，并通过 Remote_GetState() 核对摇杆和按键。
4. 连续发送超过 32 帧，验证 256 字节接收缓冲区回绕后计数仍增长。
5. 断开无线链路，确认 Remote_IsOnline(timeout_ms) 超时返回 0；恢复连接后确认重新收到数据。
6. 检查 uart_list[BSP_UART_REMOTE].rx_overruns；正常运行应不增长。环形缓冲区不能替代任务及时调度。

本次未进行实物无线联调。
