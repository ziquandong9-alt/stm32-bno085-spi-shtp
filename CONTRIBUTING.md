# Contributing / 参与贡献

感谢你帮助改进这个 BNO085 驱动。本项目优先保证协议正确性、硬件可复现性和跨 MCU 可移植性。

## 提交问题前

请先阅读：

- `README.md`：接线、构建和使用方式。
- `docs/DEBUGGING.zh-CN.md`：常见错误现象。
- `docs/DRIVER_WALKTHROUGH.zh-CN.md`：驱动调用链与协议说明。

硬件问题至少提供：

- MCU、开发板和 BNO085 模块型号。
- BNO085 part number、固件版本和 reset cause。
- SPI 模式、实际 SCK 频率和供电电压。
- PS0、PS1、BOOTN、CLKSEL 接法。
- 完整串口日志；不要只截最后一行。
- 发布前删除日志中的用户名、绝对路径、调试器序列号和其他隐私信息。
- 若可用，附 H_INTN、CS、SCK、MOSI、MISO 的逻辑分析仪波形。

## 代码分层规则

- `BNO085_Driver/Src/bno085.c` 不得包含 MCU 厂商 HAL 头文件。
- 新平台代码放入 `BNO085_Driver/Port/<platform>`。
- 仅应用真正需要的 API、类型和事件宏放入 `bno085.h`。
- 协议内部宏和辅助函数留在 `.c`，辅助函数使用 `static`。
- 一次 SHTP 事务期间不得释放 CS。
- SPI 读取 dummy byte 必须为 `0x00`。
- 不使用动态内存；如需改变该约束，请先说明使用场景和 RAM 影响。

## 风格

- C99，兼容 ARM Compiler 5.06。
- 新增关键逻辑时使用简洁的中英双语注释。
- 公共 API 使用 `BNO085_` 前缀，平台接口使用 `BNO085_Port_` 前缀。
- 所有输入指针都要检查 `NULL`。
- timeout 计算使用无符号差值，允许毫秒时基自然回卷。

## 提交前验证

至少完成：

1. Keil Rebuild：0 error、0 warning。
2. STM32CubeProgrammer 下载并 verify 成功。
3. 串口确认 Product ID、accelerometer 和 Rotation Vector 均启动。
4. 连续运行至少 60 秒，无 timeout、invalid report 或自动重启。
5. `git diff --check` 无空白错误。

Pull Request 应说明改动目的、测试硬件、测试结果和是否改变公共 API。

## English summary

Keep the portable core vendor-HAL-free, put board-specific code under `Port`, preserve
SHTP transaction boundaries, and include exact hardware/build/runtime validation in every
change. New non-trivial code should include concise bilingual comments.
