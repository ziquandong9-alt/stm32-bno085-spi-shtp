# 跟着代码学会 BNO085 SPI 驱动

本文按最终工程的真实调用顺序讲解。建议同时打开以下文件：

- [`BNO085_Driver/Inc/bno085.h`](../BNO085_Driver/Inc/bno085.h)
- [`BNO085_Driver/Src/bno085.c`](../BNO085_Driver/Src/bno085.c)
- [`BNO085_Driver/Port/bno085_port.h`](../BNO085_Driver/Port/bno085_port.h)
- [`BNO085_Driver/Port/STM32/bno085_port_stm32.c`](../BNO085_Driver/Port/STM32/bno085_port_stm32.c)
- [`Core/Src/main.c`](../Core/Src/main.c)

学习目标不是背 API，而是能回答三个问题：

1. 主机为什么必须看 H_INTN，不能像普通 SPI 寄存器那样随时读？
2. 一个 SHTP 包怎样变成 yaw、roll、pitch 和三轴加速度？
3. 换 MCU 时为什么只改 Port 层就够了？

---

## 第 0 步：先建立正确的分层模型

BNO085 内部已经有 MCU 和 SH-2 融合算法。STM32 并不直接读取 MEMS 寄存器，而是在和另一个
处理器交换消息。

```text
应用 main.c
    |
    | BNO085_Init / Enable / Poll / GetYaw ...
    v
bno085.c                 只处理 SHTP/SH-2
    |
    | BNO085_Port_*
    v
bno085_port_stm32.c      把抽象操作翻译成 STM32 HAL
    |
    v
SPI1 + CS/WAKE/RESET/INT
```

因此，`bno085.c` 里不应该出现 `SPI_HandleTypeDef`、`GPIO_TypeDef` 或 `HAL_*`。这条边界是判断
驱动是否真正可移植的第一条标准。

---

## 第 1 步：配置 SPI 和四根控制线

打开 `Core/Src/spi.c`，关键配置是：

```c
hspi1.Init.Mode = SPI_MODE_MASTER;
hspi1.Init.Direction = SPI_DIRECTION_2LINES;
hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
```

逐项理解：

- `CPOL=HIGH`、`CPHA=2EDGE` 对应 SPI Mode 3。
- APB2 为 84 MHz，32 分频得到 2.625 MHz。
- BNO085 上限是 3 MHz，所以不能改成 16 分频；84/16=5.25 MHz，已经超规格。
- 使用软件 NSS，因为 CS 的持续时间必须由驱动精确控制。

除 SCK/MISO/MOSI 外还有四根信号：

| 信号 | 方向 | 有效电平 | 作用 |
|---|---|---:|---|
| H_CSN | STM32 -> BNO085 | 低 | 包围一次完整 SPI 事务 |
| WAKE/PS0 | STM32 -> BNO085 | 低 | 写事务前唤醒/握手；复位采样时又是模式选择 |
| NRST | STM32 -> BNO085 | 低 | 硬件复位 |
| H_INTN | BNO085 -> STM32 | 低 | 有数据，或已响应主机 WAKE |

最容易混淆的是 WAKE/PS0：复位期间它作为 PS0，必须保持高并配合 PS1=高选择 SPI；启动后它
才作为低有效 WAKE 使用。

---

## 第 2 步：把 STM32 硬件交给 Port 层

在 `main.c` 的 `BNO085_Start()` 中，应用填写：

```c
BNO085_STM32_PortConfig_t port;

port.spi = &hspi1;
port.cs_port = BNO085_CS_GPIO_Port;
port.cs_pin = BNO085_CS_Pin;
port.wake_port = BNO085_WAKE_GPIO_Port;
port.wake_pin = BNO085_WAKE_Pin;
port.reset_port = BNO085_RST_GPIO_Port;
port.reset_pin = BNO085_RST_Pin;
port.interrupt_port = BNO085_INT_GPIO_Port;
port.interrupt_pin = BNO085_INT_Pin;

BNO085_STM32_Port_Init(&port);
```

`BNO085_STM32_Port_Init()` 会复制整个结构体，不会保存局部变量 `port` 的地址，所以函数返回后
不会产生悬空指针。

随后 Port 层提供八个操作。例如：

```c
bool BNO085_Port_SPITransfer(const uint8_t *tx, uint8_t *rx,
                            uint16_t length, uint32_t timeout_ms)
{
    return HAL_SPI_TransmitReceive(port_config.spi,
                                   (uint8_t *)tx,
                                   rx,
                                   length,
                                   timeout_ms) == HAL_OK;
}
```

核心只知道“全双工传输 length 字节”，不知道 STM32 的句柄类型。换成 GD32、ESP32 或裸机驱动时，
只要实现同样语义即可。

Port 层还有一个重要约定：

```c
BNO085_Port_SetChipSelect(true);   /* true = asserted = 拉低 */
BNO085_Port_SetChipSelect(false);  /* false = deasserted = 拉高 */
```

这让核心表达的是“信号是否有效”，而不是绑定某款 MCU 的 GPIO_SET/GPIO_RESET 枚举。

---

## 第 3 步：从 `BNO085_Init()` 进入完整复位流程

`BNO085_Init()` 很短：

```c
BNO085_Status_t BNO085_Init(void)
{
    if (!BNO085_Port_IsReady()) {
        return BNO085_ERR_PORT;
    }
    return BNO085_Reset();
}
```

真正的初始化在 `BNO085_Reset()`：

```c
initialized = false;
have_rotation_vector = false;
have_euler = false;
have_acceleration = false;
memset(tx_sequence, 0, sizeof(tx_sequence));

BNO085_Port_SetChipSelect(false);
BNO085_Port_SetWake(false);   /* false=释放，即 WAKE/PS0 为高 */
BNO085_Port_SetReset(true);   /* true=有效，即 NRST 为低 */
BNO085_Port_DelayMs(10U);
BNO085_Port_SetReset(false);  /* 释放复位 */

status = wait_for_advertisement();
```

为什么复位前要清缓存和 sequence？

- 复位后的设备 sequence 从头开始，主机也必须同步清零。
- 如果不清 `have_*`，应用可能在新数据到来前误读上一次运行留下的姿态。
- `initialized` 只有在两个启动阶段全部完成后才变成 `true`。

---

## 第 4 步：理解 `wait_for_interrupt()`

读事务和写事务都要看 H_INTN，但含义稍有不同：

```c
static BNO085_Status_t wait_for_interrupt(uint32_t timeout_ms,
                                          bool assert_wake)
```

- `assert_wake=false`：设备自己把 H_INTN 拉低，表示有数据可读。
- `assert_wake=true`：主机先把 WAKE 拉低，设备再把 H_INTN 拉低，表示已准备接收写包。

核心循环读取低有效中断：

```c
while (!BNO085_Port_IsInterruptAsserted()) {
    if (timed_out(start_ms, timeout_ms)) {
        ...
        return BNO085_ERR_TIMEOUT;
    }
}
```

`timed_out()` 使用无符号减法：

```c
(uint32_t)(now - start) >= timeout
```

即使 32 位毫秒计数从 `0xFFFFFFFF` 回卷到 0，只要超时窗口小于半个计数范围，这种写法仍然成立。

---

## 第 5 步：理解 SHTP 四字节包头

每个 SHTP 物理包都以四字节开头：

```text
byte 0  Length LSB
byte 1  Length MSB，其中 bit15 是 continuation 标志
byte 2  Channel
byte 3  Sequence number
```

长度包含包头本身。例如发送 17 字节 Set Feature cargo，总长度是 21，即十六进制 `0x0015`：

```text
15 00 02 00 | FD 05 ...
^length     ^ channel 2  ^ cargo
```

接收时：

```c
raw_length = read_u16_le(header);
packet_length = (uint16_t)(raw_length & 0x7FFFU);
*cargo_length = packet_length - SHTP_HEADER_SIZE;
*channel = header[2];
*continuation = (header[1] & 0x80U) != 0U;
```

`0x7FFF` 用来去掉 bit15 continuation 标志。`0xFFFF` 被协议保留为非法长度，因此必须明确拒绝。

通道用途：

- channel 0：SHTP command/advertisement。
- channel 1：executable，例如 reset complete。
- channel 2：SH-2 control，例如 Product ID、Set Feature。
- channel 3：non-wake sensor reports，本工程的姿态和加速度都从这里来。

---

## 第 6 步：为什么 SPI 读取必须发送 0x00

SPI 没有“只读”动作。要接收一个字节，主机必须同时发送一个字节。

`spi_read_bytes()` 的核心是：

```c
memset(io_tx, 0, count);
BNO085_Port_SPITransfer(io_tx, destination, count, SPI_TIMEOUT_MS);
```

这些 0 不是随意的 dummy data。BNO085 同时观察 MOSI，并把最先收到的两个字节解释成主机方向
SHTP 长度：

- `00 00` 表示主机没有 cargo，仅产生读取时钟。
- `FF FF` 表示非法保留长度 `0xFFFF`，会触发协议错误。

这是本次驱动最关键的修复之一。

---

## 第 7 步：`read_packet()` 怎样保持包边界

调用过程如下：

```text
等 H_INTN 低
  -> CS 拉低
  -> 读 4 字节 header
  -> 从 header 得到 cargo 长度
  -> CS 保持低，继续读完整 cargo
  -> CS 拉高
```

对应代码：

```c
BNO085_Port_SetChipSelect(true);
status = spi_read_bytes(header, sizeof(header));
...
status = spi_read_bytes(buffer, copy_length);
...
BNO085_Port_SetChipSelect(false);
```

注意 header 和 cargo 之间没有释放 CS。虽然底层调用了两次 HAL 传输，但对 BNO085 而言仍是同一个
CS 低电平窗口。

如果设备报告 300 字节，而本地只容纳 256 字节，不能在读满 256 字节后直接拉高 CS。代码会继续：

```c
spi_read_bytes(NULL, cargo_length - copy_length);
```

多余内容被丢弃，但仍从 SPI 线上完整读走。否则下一次读取会从上一个包中间开始，之后每个包头都会
错位。

---

## 第 8 步：为什么启动要等待两个消息

`wait_for_advertisement()` 第一次调用 `read_packet()`，要求：

```c
channel == CHANNEL_COMMAND       /* channel 0 */
cargo_length > 0
continuation == false
```

这只是 SHTP advertisement，证明传输层存活。

随后继续收包，直到：

```c
channel == CHANNEL_EXECUTABLE    /* channel 1 */
cargo_buffer[0] == 1U            /* reset complete */
```

只有这时 `BNO085_Reset()` 才设置：

```c
initialized = true;
```

如果省略第二阶段，Product ID 偶尔可能成功，但紧接着的 Set Feature 可能被 SH-2 忽略。这正是早期
出现“能打印固件版本，却没有 rotation data”的原因。

---

## 第 9 步：主机怎样发送一个 SHTP 包

以 `send_packet(data, length, channel)` 为中心看写流程。

### 9.1 构造包头

```c
packet_length = length + 4U;
header[0] = packet_length & 0xFFU;
header[1] = (packet_length >> 8) & 0x7FU;
header[2] = channel;
header[3] = tx_sequence[channel];
```

`tx_sequence` 是数组而不是单个变量，因为 SHTP 要求每个通道独立计数。

### 9.2 清理旧输入

设备正在以 100 Hz 输出时，H_INTN 低可能代表“有传感器包”，而不是“响应本次 WAKE”。因此写入前
调用 `drain_pending_packets()`，直到 H_INTN 回到高电平。

### 9.3 发起写握手并连续发送

```c
wait_for_interrupt(COMMAND_TIMEOUT_MS, true); /* 拉低 WAKE，等 H_INTN */
BNO085_Port_SetChipSelect(true);
BNO085_Port_SetWake(false);
tx_sequence[channel]++;
spi_write_packet(header, data, length);
BNO085_Port_SetChipSelect(false);
```

`spi_write_packet()` 先把 header 和 cargo 拼进 `io_tx`，再调用一次 SPI transfer。写包绝不能把 header
和 cargo 分成两个 CS 窗口。

---

## 第 10 步：Product ID 为什么要收四条响应

`BNO085_GetProductInfo()` 发送：

```c
uint8_t request[2] = { 0xF9U, 0U };
send_packet(request, sizeof(request), CHANNEL_CONTROL);
```

然后在 channel 2 中寻找 report ID `0xF8`。第一条响应的字段解析为：

```c
info->reset_cause    = p[1];
info->sw_major       = p[2];
info->sw_minor       = p[3];
info->sw_part_number = read_u32_le(p + 4U);
info->build_number   = read_u32_le(p + 8U);
info->sw_patch       = read_u16_le(p + 12U);
```

API 保存第一条，但 `response_count` 必须达到 4 才返回。这个“读完所有响应再发下一命令”的习惯对
消息型设备很重要。

---

## 第 11 步：Set Feature 怎样启用 100 Hz 报告

公开 API：

```c
BNO085_EnableAccelerometer(10000U);
BNO085_EnableRotationVector(10000U);
```

它们最终进入 `set_report_interval()`。17 字节 cargo 的关键位置：

```c
command[0] = 0xFDU;       /* Set Feature */
command[1] = report_id;   /* 0x01 accel，0x05 rotation vector */
command[5] = interval_us & 0xFFU;
command[6] = interval_us >> 8;
command[7] = interval_us >> 16;
command[8] = interval_us >> 24;
```

`10000` 十六进制为 `0x00002710`，按小端排列在 byte 5～8：

```text
10 27 00 00
```

Set Feature 本身是单向控制命令。因此示例随后不是等一个“enable OK”字符串，而是真正调用
`BNO085_Poll()`，直到同时收到 acceleration 和 rotation-vector event。这是更可靠的功能确认。

---

## 第 12 步：`BNO085_Poll()` 是流式阶段唯一 I/O 入口

主循环只做：

```c
uint32_t events = 0U;
status = BNO085_Poll(200U, &events);
```

`BNO085_Poll()` 的内部调用链：

```text
BNO085_Poll
  -> receive_channel(channel 3)
      -> read_packet
          -> wait_for_interrupt
          -> spi_read_bytes(header)
          -> spi_read_bytes(cargo)
  -> parse_sensor_payload
      -> 更新缓存
      -> 设置 event bits
```

为什么返回 event bits，而不直接返回某一种数据？因为同一个 SHTP cargo 可能同时包含时间戳、
加速度和 Rotation Vector。位图允许一次调用告诉应用多个缓存已更新：

```c
if (events & BNO085_EVENT_ROTATION_VECTOR) { ... }
if (events & BNO085_EVENT_ACCELEROMETER)   { ... }
```

---

## 第 13 步：怎样遍历一个 cargo 中的多个报告

channel 3 cargo 不是“一包只放一个传感器值”。常见布局类似：

```text
FB + 4-byte base timestamp
01 + accelerometer fields
05 + rotation-vector fields
```

`parse_sensor_payload()` 使用 cursor：

```c
while (cursor < length) {
    item_length = report_length(payload[cursor]);
    ...解析 payload + cursor...
    cursor += item_length;
}
```

`report_length()` 是必要的，因为各报告没有统一长度。遇到未知 ID 或剩余字节不足一个完整报告时，
立即返回 `BNO085_ERR_INVALID_REPORT`，而不是猜测下一个位置。

`0xFB` Base Timestamp 提供本 cargo 的传感器时间基准。当前驱动把它保存到同一 cargo 后续样本的
`timestamp_us`；没有进一步应用每个子报告 byte 3 的 delay 修正。如果将来做高精度多传感器时间同步，
可以在这里扩展，而不需要修改 SPI 层。

---

## 第 14 步：解析三轴加速度

accelerometer 报告长度为 10 字节：

```text
byte 0     report ID = 0x01
byte 1     sensor sequence
byte 2     status，低两位是 accuracy 0~3
byte 3     delay
byte 4..5  X，signed Q8
byte 6..7  Y，signed Q8
byte 8..9  Z，signed Q8
```

Q8 表示整数值中有 8 个小数位，因此换算是：

```c
float_value = signed_raw / 2^8 = signed_raw / 256
```

代码写成预计算常量乘法：

```c
#define Q8_SCALE (1.0f / 256.0f)
latest_acceleration.x_mps2 = read_s16_le(p + 4U) * Q8_SCALE;
```

解析完成后：

```c
have_acceleration = true;
*events |= BNO085_EVENT_ACCELEROMETER;
```

这里的值包含重力。传感器静止时三个轴的矢量模长应接近 `9.81 m/s²`，不应该三个轴都接近 0。

---

## 第 15 步：解析 Rotation Vector 四元数

Rotation Vector 报告长度为 14 字节：

```text
byte 0      report ID = 0x05
byte 1      sequence
byte 2      status/accuracy
byte 3      delay
byte 4..5   i = x，signed Q14
byte 6..7   j = y，signed Q14
byte 8..9   k = z，signed Q14
byte 10..11 real = w，signed Q14
byte 12..13 angular accuracy，unsigned Q12 rad
```

换算：

```c
x = read_s16_le(p + 4U)  / 16384.0f;
y = read_s16_le(p + 6U)  / 16384.0f;
z = read_s16_le(p + 8U)  / 16384.0f;
w = read_s16_le(p + 10U) / 16384.0f;
accuracy_rad = read_u16_le(p + 12U) / 4096.0f;
```

注意顺序是 `x,y,z,w`，而结构体为了数学阅读方便排列成 `w,x,y,z`。代码必须按偏移逐个赋值，不能
把 8 字节直接 memcpy 进结构体。

四元数缓存完成后立即调用一次 `quaternion_to_euler()`，然后设置 Rotation Vector event。应用调用
三个角度 Getter 时不再重复三角函数计算。

---

## 第 16 步：四元数怎样转换成 yaw、roll、pitch

工程采用 Z-Y-X 欧拉角：先理解为 yaw 绕 Z、pitch 绕 Y、roll 绕 X。代码为：

```c
sin_roll  = 2 * (w*x + y*z);
cos_roll  = 1 - 2 * (x*x + y*y);
sin_pitch = 2 * (w*y - z*x);
sin_yaw   = 2 * (w*z + x*y);
cos_yaw   = 1 - 2 * (y*y + z*z);

roll  = atan2(sin_roll, cos_roll);
pitch = asin(clamp(sin_pitch, -1, 1));
yaw   = atan2(sin_yaw, cos_yaw);
```

结果先是弧度，再乘 `180/pi` 变成度。

欧拉角存在数学上的万向锁：pitch 接近 ±90° 时，yaw 和 roll 会强烈耦合。这不是驱动错误。如果控制
算法需要全姿态运算，优先直接使用 `BNO085_GetRotationVector()` 返回的四元数。

---

## 第 17 步：Getter 为什么不会再次访问 SPI

例如：

```c
BNO085_Status_t BNO085_GetYaw(float *yaw_deg)
{
    if (yaw_deg == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_euler) return BNO085_ERR_NO_DATA;
    *yaw_deg = latest_euler.yaw_deg;
    return BNO085_OK;
}
```

它只做三件事：检查指针、检查是否已有首帧、复制一个 float。Roll、Pitch 和加速度 Getter 相同。

所以正确使用模式是：

```c
status = BNO085_Poll(200U, &events);  /* 这里发生 SPI I/O */
if ((status == BNO085_OK) &&
    (events & BNO085_EVENT_ROTATION_VECTOR)) {
    BNO085_GetYaw(&yaw);              /* 以下都不发生 SPI I/O */
    BNO085_GetRoll(&roll);
    BNO085_GetPitch(&pitch);
}
```

不应该只反复调用 Getter 而不调用 `BNO085_Poll()`，否则读到的永远是同一份旧缓存。

---

## 第 18 步：读懂 `main()` 的持续输出和自动恢复

启动成功后，主循环每次最多等待 200 ms：

```c
BNO085_Status_t status = BNO085_Poll(200U, &events);
```

只有本次出现 Rotation Vector event 才打印 YPR。加速度可能单独到达，此时缓存会更新，但不会产生
一行重复姿态输出。

每次姿态成功时记录：

```c
last_rotation_ms = HAL_GetTick();
```

若超过一秒没有新姿态：

```c
if ((uint32_t)(HAL_GetTick() - last_rotation_ms) >= 1000U) {
    BNO085_Start();
}
```

恢复依据特意不是“是否收到任何数据”。即使 accelerometer 还在输出，只要 Rotation Vector 停止，
应用仍会重新初始化。

---

## 第 19 步：看到错误时从哪一层查

| 状态 | 首先检查 |
|---|---|
| `platform I/O error` | HAL SPI 返回值、SPI overrun、线缆和供电 |
| `timeout` during init | H_INTN、WAKE/PS0、PS1、NRST 和启动等待顺序 |
| `invalid report` | SPI Mode、dummy byte、CS 是否中途释放、包是否排空 |
| Product ID 正常但无数据 | 是否等待 reset complete、Set Feature 周期和 report ID |
| `no cached data` | 是否先启用报告并成功调用过 `BNO085_Poll()` |
| YPR 跳变 | 安装坐标、磁干扰、欧拉角万向锁；先查看原始四元数 |

把问题定位到“Port/SPI”“SHTP”“SH-2 配置”“数学/坐标”中的一层，会比同时改所有参数有效得多。

---

## 第 20 步：你可以按这组练习确认已经学会

1. 把报告周期改为 `20000U`，解释为什么输出约变成 50 Hz。
2. 在主循环读取 `BNO085_GetAcceleration()`，静止时验证模长接近 9.81。
3. 暂时把 SPI 改回 128 分频，比较功能不变但总线占用增加多少。
4. 在 `read_packet()` 中打印 header 的四个字节，观察不同 channel 的 sequence 独立变化。
5. 新建一个假的 PC Port 层，用预先保存的 cargo 喂给解析器，为 Q8/Q14 换算写单元测试。
6. 最后尝试 H_INTN EXTI + SPI DMA，但必须保持一次 SHTP 事务中 CS 连续为低。

完成前四项，你已经能独立读懂和维护这个驱动；完成第五项，就具备把协议解析从硬件测试中分离出来
的能力；完成第六项，则可以继续向低延迟、低 CPU 占用版本演进。
