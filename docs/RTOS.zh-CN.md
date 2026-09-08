# RTOS 接入指南

驱动核心不调用任何 RTOS API，也不创建任务，因此可同时适配 FreeRTOS、ThreadX、Zephyr 和裸机。

推荐由一个传感器任务独占驱动：H_INTN 外部中断只通知任务；任务持续调用
`BNO085_Process()`，直到返回 `BNO085_PENDING`；数据回调把完整缓存快照写入长度为 1 的队列；
控制任务只消费最新快照，不直接占用 SPI。若 Port 层没有在 DMA 完成时再次通知任务，可设置约
2 ms 的任务等待超时。

参考 `examples/freertos`。不要在 EXTI 中断中解析 SHTP，也不要让多个任务并发调用 Poll/Process。
Set Feature、保存校准和 Tare 等命令同样应通过传感器任务串行执行。

低功耗应用可为指定报告设置 `BNO085_FEATURE_WAKEUP_ENABLED`，在允许延迟时配置 batch interval，
再由 RTOS/板级策略决定 MCU 休眠。驱动核心不会强制执行 `WFI`。
