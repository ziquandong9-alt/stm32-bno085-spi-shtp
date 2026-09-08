# BNO085 驱动移植指南

## 分层边界

驱动分成三部分：

1. `Inc/bno085.h`：应用层 API 和数据结构。
2. `Src/bno085.c`：SHTP/SH-2 协议、报告解析、缓存和欧拉角换算。
3. `Port`：SPI、GPIO、延时和毫秒时基适配。

核心层不包含任何 STM32 头文件。换成其他 MCU 或 RTOS 时，不要修改 `bno085.c`，
只需为新平台实现 `bno085_port.h` 声明的接口。

最快的方式是编译 `Port/Callbacks/bno085_port_callbacks.c`，再用目标 SDK 的函数填充
`BNO085_CallbackPortConfig_t`。追求最低调用开销或需要深度控制 IRQ/DMA 时，可复制
`Port/Template/bno085_port_template.c` 到新的平台目录并直接实现接口。模板保留了故意的
`#error`，避免尚未实现的 Port 被误发布。

## 必须实现的接口

| 接口 | 约定 |
|---|---|
| `BNO085_Port_IsReady` | 平台初始化完成后返回 `true` |
| `BNO085_Port_SPITransfer` | 阻塞式全双工 SPI；成功返回 `true` |
| `BNO085_Port_SPITransferAsync` | 启动一次中断或 DMA 全双工 SPI，不等待完成 |
| `BNO085_Port_SPITransferAsyncStatus` | 返回异步 SPI 的 idle/busy/complete/error 状态 |
| `BNO085_Port_SPITransferAsyncAbort` | 复位或恢复时终止未完成异步传输 |
| `BNO085_Port_GetLastError` | 返回归一化错误和可选的厂商原始错误位 |
| `BNO085_Port_Recover` | 释放总线并重新初始化 SPI/DMA，不复位 BNO085 |
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
4. 简单阻塞式任务循环调用 `BNO085_Poll()`；高刷新率任务在
   H_INTN/DMA 唤醒后调用 `BNO085_PollAsync()` 推进状态机。
5. 首次上板先用较低 SPI 频率验证，再逐步提高，但不得超过 3 MHz。

驱动当前是无动态内存的单实例实现，不支持在多个任务或中断中同时调用。RTOS 中应
让一个任务独占 `BNO085_Poll()` 或 `BNO085_PollAsync()`，其他任务通过队列
或对缓存快照加锁来取数据。

## 进一步提速

- 将 H_INTN 配成下降沿 EXTI；是否使用 `WFI`、RTOS 信号量或循环调度由上层应用决定。
- 异步实现必须保证一个 SHTP 物理包的 4 字节头和 cargo 期间 CS 始终为低。
- 将串口日志改为 DMA/环形缓冲，或者降低日志频率。
- 只启用业务真正需要的 SH-2 报告，避免无用的总线和解析负载。
