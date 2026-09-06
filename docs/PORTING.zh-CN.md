# BNO085 驱动移植指南

## 分层边界

驱动分成三部分：

1. `Inc/bno085.h`：应用层 API 和数据结构。
2. `Src/bno085.c`：SHTP/SH-2 协议、报告解析、缓存和欧拉角换算。
3. `Port`：SPI、GPIO、延时和毫秒时基适配。

核心层不包含任何 STM32 头文件。换成其他 MCU 或 RTOS 时，不要修改 `bno085.c`，
只需为新平台实现 `bno085_port.h` 声明的接口。

## 必须实现的接口

| 接口 | 约定 |
|---|---|
| `BNO085_Port_IsReady` | 平台初始化完成后返回 `true` |
| `BNO085_Port_SPITransfer` | 阻塞式全双工 SPI；成功返回 `true` |
| `BNO085_Port_SetChipSelect` | `true` 将 H_CSN 拉低 |
| `BNO085_Port_SetWake` | `true` 将 WAKE/PS0 拉低 |
| `BNO085_Port_SetReset` | `true` 将 NRST 拉低 |
| `BNO085_Port_IsInterruptAsserted` | H_INTN 为低时返回 `true` |
| `BNO085_Port_GetTimeMs` | 可自然回卷的单调毫秒计数 |
| `BNO085_Port_DelayMs` | 毫秒阻塞延时 |

SPI 必须使用 Mode 3（CPOL=1、CPHA=1）、MSB first，时钟不能超过 3 MHz。读取时
MOSI dummy byte 必须为 `0x00`；该细节已由核心层处理。

## 移植步骤

1. 复制 `bno085_port_stm32.h/.c` 为新平台文件并替换其中的 HAL 类型和调用。
2. 在应用初始化 SPI/GPIO/时基后，调用新平台的初始化函数。
3. 调用 `BNO085_Init()`，再启用所需报告。
4. 循环调用 `BNO085_Poll()`；只在对应 event bit 出现时读取 Getter。
5. 首次上板先用较低 SPI 频率验证，再逐步提高，但不得超过 3 MHz。

驱动当前是无动态内存的单实例实现，不支持在多个任务或中断中同时调用。RTOS 中应
让一个任务独占 `BNO085_Poll()`，其他任务通过队列或对缓存快照加锁来取数据。

## 进一步提速

- 将 H_INTN 配成下降沿 EXTI，避免主循环轮询数据就绪。
- 在平台层增加 SPI DMA 时，要保证一次 SHTP 事务期间 CS 始终保持为低。
- 将串口日志改为 DMA/环形缓冲，或者降低日志频率。
- 只启用业务真正需要的 SH-2 报告，避免无用的总线和解析负载。
