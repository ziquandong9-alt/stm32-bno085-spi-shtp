# 配置与事件 API

项目把配置分成三层，换板卡时无需修改协议核心。

## 1. 硬件映射

引脚、SPI/DMA 资源和 SDK 错误码属于 Port 层。STM32 应用填写
`BNO085_STM32_PortConfig_t`；其他平台可以直接实现 `bno085_port.h`，也可以使用
`Port/Callbacks` 下的函数表适配器。

## 2. 驱动运行策略

始终先取得默认配置，只修改目标平台确实需要调整的参数：

```c
BNO085_Config_t driver;
BNO085_GetDefaultConfig(&driver);
driver.spi_timeout_ms = 50U;
driver.feature_retry_count = 3U;
BNO085_InitWithConfig(&driver);
```

超时、排空上限和重试次数不能为零。该结构体故意不包含引脚或芯片厂商类型。

## 3. SH-2 报告配置

```c
BNO085_ReportConfig_t step = {
    .interval_us = 20000U,
    .batch_interval_us = 1000000U,
    .sensor_specific = 0U,
    .change_sensitivity = 0U,
    .flags = BNO085_FEATURE_WAKEUP_ENABLED
};
BNO085_ConfigureReport(BNO085_REPORT_STEP_COUNTER, &step);
```

`interval_us` 是生成周期，`batch_interval_us` 允许 SH-2 批量缓存，
`change_sensitivity` 控制变化触发阈值，flags 可选择变化模式、唤醒、always-on 和 sniff。
具体组合是否有效取决于报告类型和固件。

通过 `BNO085_GetReportConfig()` 读取固件实际配置；固件可能量化周期。
`BNO085_FlushReport()` 要求立即输出批量样本，`BNO085_DisableReport()` 用零周期关闭报告。
Wake Report 位于 SHTP channel 4，阻塞和异步接收路径均已支持 channel 3/4。

## Poll 与回调

小型裸机程序仍可直接使用 Poll API。`BNO085_Process()` 在不改变执行上下文所有权的前提下，
增加可选数据和错误回调。回调同步运行在 `BNO085_Process()` 的调用者上下文，必须短小，且不能
递归调用 Poll/Process；Getter 只复制缓存，可以安全使用。
