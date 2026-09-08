# Changelog

本项目遵循 [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) 的结构。

## [Unreleased]

## [0.2.0] - 2026-09-08

### Added

- Calibrated gyroscope、magnetometer 和 Game Rotation Vector 报告解析与 Getter。
- STM32 SPI DMA 平台适配 API 和 `BNO085_PollAsync()` 非阻塞收包状态机。
- DMA 报告速率与核心处理 CPU 时间的实机统计。
- Get Feature 配置读回、SHTP/传感器双层序号诊断和 SPI/DMA 分级恢复。
- Base Timestamp、Timestamp Rebase 与指数编码 report delay 采样时间修正。
- Linear Acceleration、Gravity、uncalibrated gyro/magnetometer 和 Raw ADC API。
- 动态校准、Save DCD 与 Tare 命令响应匹配 API。
- PC 端解析/故障注入回归测试及 GitHub Actions。
- 统一 `BNO085_Config_t` 运行策略和完整 `BNO085_ReportConfig_t` Set Feature API。
- `BNO085_Process()` 可选数据/错误回调，并保留原 Poll API 兼容性。
- Tap、Step Counter、Step Detector 和 Stability Classifier 报告。
- channel 4 Wake Report、batch interval、change sensitivity 和 flush API。
- SDK 无关 Callback Port、新平台模板、CMake 安装导出与 FreeRTOS 示例。
- Doxygen API 文档和 GitHub Pages 自动部署工作流。
- GCC/Clang Sanitizer 测试矩阵、Cortex-M4 Arm-GCC 交叉编译和 CMake 安装包消费者测试。
- 20,000 组确定性异常输入扫描，以及截断报告、未知报告和极端时间戳回归测试。
- 标准 `BNO085Config.cmake`/`BNO085ConfigVersion.cmake` 安装包支持。

### Fixed

- 将 Base Timestamp/Rebase 中间运算提升为 64 位，避免 `INT32_MIN` 取负和累加的有符号溢出。

### Changed

- 示例同时请求 RV/Game RV/Accelerometer/Gyroscope 100 Hz 和 Magnetometer 25 Hz。
- 将事件、打印、统计和恢复逻辑封装为静态 `bno085_process()`。
- 取消示例内强制 `WFI`；DMA 未就绪时立即返回，休眠策略由上层应用决定。
- 公共 API 版本提升为 `0.2.0`，示例统一通过 `BNO085_Process()` 推进流式状态机。
- 主机测试不再直接包含 `bno085.c`；生产驱动与测试代码分别编译。
- Issue 模板增加本机路径、用户名和调试器序列号脱敏提醒。

## [0.1.0] - 2026-09-06

### Added

- BNO085 SPI Mode 3 SHTP/SH-2 驱动。
- Product ID、Rotation Vector 和 calibrated accelerometer 支持。
- Yaw、Roll、Pitch、四元数及 X/Y/Z 加速度缓存 Getter。
- MCU 无关核心、通用 Port 契约和 STM32F4 HAL 适配层。
- 初始化失败和姿态数据中断后的自动恢复。
- 中英双语源码注释、开发复盘、逐代码教学、移植与排错文档。
- GitHub Issue/PR 模板和发布前检查表。

### Fixed

- SPI 读取 dummy byte 使用 `0x00`，避免产生非法 `0xFFFF` SHTP 主机包头。
- 等待 executable channel 的 reset-complete 后才发送 SH-2 配置。
- Product ID 的四条响应全部消费，避免残留包影响后续命令。
- 缓冲不足时仍排空完整 cargo，防止后续包边界错位。
- 每个 SHTP channel 独立维护发送 sequence number。

### Changed

- SPI1 从 656.25 kHz 提升至 2.625 MHz，保持低于 BNO085 3 MHz 上限。
- 解析结果按完整帧缓存，Getter 不再重复执行 SPI I/O 或欧拉角计算。
