# BNO085 SPI 驱动编写流程复盘

这份文档只讲“这次驱动是怎么一步步做出来的”。如果你想逐函数学习最终代码，继续看
[DRIVER_WALKTHROUGH.zh-CN.md](DRIVER_WALKTHROUGH.zh-CN.md)。更细的故障现象和排错记录见
[DEBUGGING.zh-CN.md](DEBUGGING.zh-CN.md)。

## 1. 先确认它不是寄存器型 SPI 传感器

普通陀螺仪常见流程是：片选拉低，发送寄存器地址，再读固定数量字节。BNO085 不这样工作。
它内部还有一个运行 SH-2 固件的 MCU，主机看到的是 SHTP 消息通道：

```text
GPIO 握手 -> SPI 物理传输 -> SHTP 包 -> SH-2 报告 -> 姿态/加速度
```

因此最初的设计目标不是“读某个寄存器”，而是先实现四件事：

1. H_INTN、WAKE、CS、NRST 的时序。
2. 四字节 SHTP 包头。
3. 按通道独立递增的 sequence number。
4. 一个 cargo 内多个 SH-2 子报告的遍历。

## 2. 先打通最小闭环：复位和 Product ID

第一阶段没有急着解析姿态，只实现：

```text
硬复位
  -> 等 H_INTN
  -> 读 channel 0 advertisement
  -> 等 channel 1 reset complete
  -> channel 2 发送 Product ID Request
  -> 读取 Product ID Response
```

串口稳定打印下面这行，才说明 SPI 的双向传输、GPIO 握手、SHTP 长度和控制通道基本正确：

```text
BNO085 part 10004148, FW 3.2.13, build 6, reset 4
```

Product ID 阶段还暴露了一个容易忽略的点：BNO085 会返回四条 Product ID Response。API
只需要第一条，但驱动必须把四条都读完，否则后面的 Set Feature 会被残留包干扰。

## 3. 解决“命令发出但没有姿态数据”

最初可以读 Product ID，但启用 Rotation Vector 后反复出现 timeout。这说明 SPI 并非完全不通，
问题集中在“读事务内容”和“启动状态机”。最终找到两个主因。

### 3.1 SPI 读操作的 dummy byte 不能是 0xFF

SPI 全双工，主机要接收 MISO，就必须同时在 MOSI 发送字节。BNO085 会把这些 MOSI 字节也看成
主机发送方向的 SHTP 包头。

- 发送 `0xFF 0xFF ...`：设备看到长度 `0xFFFF`，这是 SHTP 明确保留的非法长度。
- 发送 `0x00 0x00 ...`：设备看到长度 0，表示主机本次没有 cargo，只是在产生读取时钟。

于是读函数固定清零发送缓冲：

```c
memset(io_tx, 0, count);
BNO085_Port_SPITransfer(io_tx, destination, count, SPI_TIMEOUT_MS);
```

这个修复消除了 channel 0 Error List 风暴。

### 3.2 advertisement 不等于 SH-2 已经启动完成

channel 0 advertisement 只证明 SHTP 传输层已经工作。真正可以发送 Set Feature 之前，还必须等
channel 1 executable 报告 `reset complete`，其 cargo 第一个字节为 `0x01`。

所以启动代码分成两个等待阶段。若在 advertisement 后立刻发 Set Feature，命令可能被静默忽略，
主机随后就只会等到 timeout。

## 4. 固化可靠的收发规则

问题定位后，把临时修改整理成几条驱动不变量：

1. 写包时，四字节 header 和 cargo 必须在同一次 CS 低电平期间发送。
2. 读包时先读 header 得到长度，再保持 CS 为低继续读完整 cargo。
3. 缓冲区即使不够，也必须把本包剩余字节全部 clock out。
4. 每个 SHTP channel 分别保存发送 sequence number。
5. 主机写入前先排空 H_INTN 指示的旧输入，再通过 WAKE/H_INTN 发起新握手。
6. 所有等待共享同一个总 timeout budget，避免嵌套等待无限延长。

这些规则分别落在 `read_packet()`、`send_packet()`、`drain_pending_packets()`、
`remaining_time()` 中。

## 5. 启用并解析 Rotation Vector

Rotation Vector 的 Set Feature 命令使用 report ID `0x05`，100 Hz 对应周期 `10000 us`。
命令发出后，驱动先用 Get Feature 读回相同 feature ID 和固件实际周期，再继续等待 channel 3
真正出现 Rotation Vector 报告。它不会把“SPI 写成功”当成“传感器启动成功”，也不会把固件合理的
周期量化误判成丢帧。

收到报告后按 SH-2 格式解码：

- `x/y/z/w`：有符号 Q14，除以 16384。
- accuracy estimate：无符号 Q12 弧度，除以 4096。
- status 低两位：精度等级 0～3。

随后按 Z-Y-X 顺序把四元数转换成 yaw、roll、pitch。`asin` 输入额外限制到 `[-1, 1]`，避免
定点数舍入造成 NaN。

## 6. 加入加速度并重新设计 API

加速度 report ID 为 `0x01`，三个轴是有符号 Q8，除以 256 后单位为 `m/s²`。这是 calibrated
accelerometer，包含重力。

为了避免应用连续调用三个 Getter 就读取三次 SPI，最终使用“一个 I/O 入口 + 多个缓存 Getter”：

```text
BNO085_Poll()
  -> 一次收包
  -> 一次遍历全部子报告
  -> 更新四元数、欧拉角、加速度缓存

BNO085_GetYaw()/GetRoll()/GetPitch()
BNO085_GetAccelerationX/Y/Z()
  -> 只复制缓存，不访问 SPI
```

这样既减少总线事务，也保证 yaw、roll、pitch 是同一帧数据。

## 7. 抽离 STM32 HAL

早期核心驱动直接保存 `SPI_HandleTypeDef` 和 GPIO 端口，换 MCU 就必须修改协议代码。重构后分为：

- `bno085.c`：只理解 SHTP/SH-2。
- `bno085_port.h`：规定核心需要的阻塞/异步 SPI、GPIO 和时基操作。
- `bno085_port_stm32.c`：唯一调用 `HAL_SPI_*`、`HAL_GPIO_*`、`HAL_GetTick()` 的文件。

这一步的判断标准很简单：在 `bno085.c`、`bno085.h` 和 `bno085_port.h` 中搜索 `HAL_` 或
`stm32`，结果必须为空。

## 8. 性能优化和边界

完成正确性后才提速：

- SPI1 从 656.25 kHz 提升到 2.625 MHz，低于 BNO085 的 3 MHz 上限。
- SPI DMA 分别读取 4 字节包头和 cargo，两段期间保持 CS 连续为低。
- `BNO085_PollAsync()` 以状态机取代流式阶段的阻塞等待。
- 大收发缓冲静态复用，降低栈占用。
- 每个 cargo 只扫描一次，每个 Rotation Vector 只计算一次欧拉角。
- Getter 只读缓存。

同步版把协议边界验证正确后，再加入 H_INTN EXTI 和 SPI DMA。
`BNO085_PollAsync()` 在 DMA 未完成时立即返回，不把上层绑定到 `WFI` 或任何
特定调度方式。下一个主要瓶颈是阻塞式 UART `printf`，控制环应用可再
换成 UART DMA 环形缓冲。

## 9. 验证顺序

每轮修改都按同一顺序收敛：

1. 静态检查：核心层没有 HAL 泄漏，私有函数均为 `static`。
2. ARM Compiler 5 全量重编译，必须 `0 Error(s), 0 Warning(s)`。
3. STM32CubeProgrammer 下载并 verify。
4. 使用调试串口抓取完整启动信息。
5. 确认 Product ID、YPR 和五种报告的每秒计数连续出现。
6. 观察是否出现 timeout、invalid report 或自动重启。

最终板上同时启用 5 种报告，在 2.625 MHz SPI DMA 下稳定获取
100 Hz Rotation Vector，示例为减少 UART 阻塞每 4 帧打印一次 YPR。

## 10. 这次最值得保留的方法

- 先建立最小可验证闭环，不要一上来同时调姿态算法、串口和 SPI。
- 用“现象属于哪一层”缩小范围：Product ID 能读到，就不要再怀疑所有接线。
- 协议驱动必须把线上的每一个字节都解释清楚，dummy byte 也属于协议输入。
- 命令发送成功与功能真正产生数据是两件事，必须用首帧数据做最终确认。
- 正确性稳定后再做抽象和性能优化，否则只会让故障跨更多层传播。
