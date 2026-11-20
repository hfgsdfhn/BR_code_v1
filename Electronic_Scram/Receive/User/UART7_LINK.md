# Receive 与 Send 的通信

Receive 只通过 USART3（PB10/PB11）连接 LoRa 模块，并接收 Send 发出的状态帧。

```text
Send USART1 / PA9 -> LoRa 空口 -> Receive USART3_RX / PB11
```

Receive 不再使用 USART1，也不再连接 Master UART7。Receive 的 USART3 接收数据在本地解析为 GPIO 状态。

串口参数为 115200、8N1、无硬件流控。Send 状态帧格式为 `A5 5A status`。
