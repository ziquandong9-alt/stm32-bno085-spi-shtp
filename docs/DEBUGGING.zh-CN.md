# BNO085 SPI 驱动开发复盘

这份记录整理了在 STM32F407ZG + HAL 上从“能读 Product ID，但没有姿态数据”到
稳定输出 100 Hz Yaw/Roll/Pitch 的定位过程。

## 1. 最初的误区：把它当成普通 SPI 寄存器器件

BNO085 的 SPI 接口传输的是 SHTP packet，不存在“读某个寄存器地址得到数据”的流程。
每次传输都有 4 字节 SHTP header：

```text
byte 0-1: packet length，包含 4 字节 header；bit 15 是 continuation
byte 2  : channel
byte 3  : sequence number
```

常用通道：

| channel | 用途 |
|---:|---|
| 0 | SHTP command / advertisement |
| 1 | executable device control / reset complete |
| 2 | SH-2 control，例如 Product ID、Set Feature |
| 3 | non-wake sensor input |
| 4 | wake sensor input |
| 5 | gyro-integrated rotation vector |

同时还要使用低有效的 `H_INTN` 作为数据就绪和主机写入握手信号。

## 2. 症状一：Product ID 正常，但 Set Feature 一直超时

串口最初可以打印：

```text
BNO085 part 10004148, FW 3.2.13, build 6, reset 4
BNO085 enable failed: timeout (1)
```

这证明 SPI 引脚、模式、基本读包和 Product ID 请求大体正确，但不能证明 SH-2
应用已经启动完毕。

### 原因

驱动收到 channel 0 advertisement 后，只等待了几毫秒总线安静，随后立即发送
Set Feature。实机 trace 后发现，SH-2 的 channel 1 `reset complete` 事件是在稍后才到达。
在它之前发送的 Set Feature 会被忽略，因此没有 Rotation Vector 数据。

### 修复

硬件复位后的状态机改为：

1. 等待并读取 channel 0 advertisement。
2. 持续处理启动包。
3. 明确等到 channel 1、payload `0x01` 的 reset-complete 响应。
4. 再请求 Product ID 和配置 sensor feature。

不要用固定的 5 ms、10 ms 延时替代协议状态；不同上电条件下固件启动时间会变化。

## 3. 症状二：channel 0 Error List 包持续出现

临时 trace 显示设备不断产生 channel 0、report ID `0x01` 的 Error List 包，sequence
持续递增。读取一个包后，`H_INTN` 很快再次拉低。

### 原因

SPI 是全双工的。主机为了读 MISO 必须同时在 MOSI 发送数据。最初的读取函数把
dummy buffer 塑造成全 `0xFF`。BNO085 同时接收到 `FF FF FF FF`，会把它解释成一个
非法的主机 SHTP header，于是每读一次就制造一次新错误，形成错误风暴。

### 修复

读取时 MOSI 使用全零 dummy buffer：

```c
memset(dummy_tx, 0x00, sizeof(dummy_tx));
HAL_SPI_TransmitReceive(&hspi1, dummy_tx, rx, length, timeout);
```

SHTP 的零长度 host transfer 表示主机此时没有 cargo。CEVA 的 STM32 示例也使用
`txZeros`。这处修改后 Error List 风暴立即消失。

## 4. 症状三：偶发 invalid report 或下一个包头错位

### 原因

如果设备声明的 cargo 大于调用者缓冲区，而驱动只读取能装下的部分就释放 CS，剩余
字节仍在当前 SHTP transaction 中。下一次读取会从旧 cargo 中间开始，普通数据就会
被当作新 header。

### 修复

每次读取都必须 clock out 整个物理 payload：

- 缓冲区能容纳的部分复制给调用者。
- 多出的部分读入丢弃缓冲区。
- 完整排空后才能释放 CS。
- 向上层返回 `BUFFER_TOO_SMALL`，但不能把 SPI 流留在半包状态。

## 5. 主机写入必须符合 H_INTN/WAKE 握手

发送一个 host packet 前：

1. 先处理所有已经挂起的 device-to-host packet，使 `H_INTN` 回到高。
2. 将 WAKE 拉低，通知设备主机准备写。
3. 等待 `H_INTN` 拉低确认。
4. CS 拉低。
5. 在同一次 transfer 中连续发送 4 字节 header 和完整 cargo。
6. 释放 WAKE 和 CS。

header 与 cargo 不应拆成互不相关、CS 中间抬高的两次 packet。

## 6. Sequence number 不是全局计数器

SHTP sequence number 按 channel 独立维护。channel 2 的 Product ID/Set Feature 与
其他 channel 不能共用一个全局 sequence。硬件复位时所有通道计数器清零。

## 7. Product ID 为什么要接收多个响应

BNO08X 通常对一次 Product ID Request 返回多个 Product ID Response。驱动保存第一个
条目用于打印，但继续消费预期的全部响应，避免未读 response 阻塞后续 host write。

## 8. Set Feature 与 Rotation Vector

Set Feature 是 channel 2 上的 17 字节 report：

```text
report ID       0xFD
feature ID      0x05 (Rotation Vector)
report interval 10000 us (100 Hz, little-endian)
```

Set Feature 没有独立 ACK。当前驱动随后发送 Get Feature (`0xFE`)，按 feature ID 匹配
Feature Response (`0xFC`) 并核对固件实际采用的周期；同时仍在合理超时内等待第一个实际
Rotation Vector report。配置读回与真实数据到达缺一不可，且应允许固件对报告周期做合理量化。

## 9. Sensor report 的解析

channel 3 cargo 常见结构是：

```text
Base Timestamp (0xFB, 5 bytes)
Sensor report 1
Sensor report 2
...
```

不能假定一个 cargo 只有一个 report。解析器必须根据 report ID 查长度并移动 cursor。
Rotation Vector `0x05` 长 14 字节：

- `i/j/k/real`：signed 16-bit，Q14
- accuracy estimate：unsigned 16-bit，Q12，单位 rad
- status 低 2 位：0/1/2/3 对应 unreliable/low/medium/high

## 10. `acc=0` 不是通信失败

后续诊断同时打开 calibrated accelerometer 和 magnetic field report：

```text
rv_acc=0 accel_acc=2 mag_acc=0
MAG: x=-10.31 y=5.50 z=-58.44 |B|=59.59 uT acc=0
```

磁场三轴随旋转变化且处于几十微特斯拉量级，证明 SPI 传输、报告偏移和磁力计硬件均
有数据。`mag_acc=0` 表示磁力计动态校准尚未完成，而不是驱动读不到数据。对只需要
连续 YPR 的示例，最终串口隐藏这些诊断字段；驱动仍保留 Rotation Vector accuracy。

需要可靠绝对航向时，应按 CEVA 流程以 50 Hz 打开 Magnetic Field、覆盖 roll/pitch/yaw
三个轴的正反旋转，并在成功后保存 DCD。若应用只需要相对姿态且磁环境不稳定，Game
Rotation Vector 更合适，但 Yaw 会有长期漂移。

## 11. 最终验证

清理 trace 后的正式版本完成以下验证：

- ARM Compiler 5：0 error、0 warning
- Keil 下载：Erase Done / Programming Done / Verify OK
- 硬件复位后稳定识别 part `10004148`、FW `3.2.13`
- 100 Hz Rotation Vector 持续输出
- 连续数秒采集数百个 YPR report，无 enable timeout 和 packet error storm

## 12. 排错速查表

| 现象 | 优先检查 |
|---|---|
| 完全没有 H_INTN | 供电、NRST、BOOTN、PS0/PS1 接口选择 |
| header 总是 `FF FF FF FF` | MISO、CS、SPI Mode 3、器件是否处于 SPI 模式 |
| Product ID 正常但无 sensor data | 是否等待 reset complete；Set Feature 长度与通道 |
| Error List 无限出现 | 读取 dummy byte 是否为 `0xFF`；包是否完整排空 |
| 偶发 invalid report | CS 边界、payload 是否读完、report cursor/长度表 |
| H_INTN 一直低，主机无法写 | 仍有 device packet 未读；先 drain 再执行 WAKE 握手 |
| YPR 正常但 accuracy 为 0 | 传感器校准和磁环境，不是 SPI 连通性 |
| Pitch 接近 ±90° 时角度跳变 | 欧拉角万向节锁，内部四元数通常仍连续 |

## 13. 参考实现

- [BNO08X Datasheet](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)
- [Sensor Hub Transport Protocol](https://www.ceva-ip.com/wp-content/uploads/Sensor-Hub-Transport-Protocol.pdf)
- [CEVA SH-2 host library](https://github.com/ceva-dsp/sh2)
- [CEVA STM32/Nucleo demonstration](https://github.com/ceva-dsp/sh2-demo-nucleo)
- [BNO08X Sensor Calibration Procedure](https://www.ceva-ip.com/wp-content/uploads/2019/09/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf)
