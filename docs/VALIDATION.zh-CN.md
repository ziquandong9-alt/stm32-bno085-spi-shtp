# 实机验证记录

验证日期：2026-09-07

## 环境

- MCU：STM32F407ZGTx
- BNO085 part：10004148
- BNO085 firmware：3.2.13，build 6
- 编译器：ARM Compiler 5.06 update 7 (build 960)
- 下载器：ST-Link V2，固件 V2J46S7
- 下载工具：STM32CubeProgrammer 2.21.0
- 板上电压：3.26 V
- SPI1：Mode 3，2.625 MHz，MSB first
- USART1：115200 8-N-1

## 构建结果

```text
Program Size: Code=25524 RO-data=628 RW-data=140 ZI-data=3100
"BNO085\BNO085.axf" - 0 Error(s), 0 Warning(s).
```

## 下载结果

```text
File download complete
Download verified successfully
MCU Reset
```

## 串口结果

```text
BNO085 demo boot (SPI DMA)
BNO085 part 10004148, FW 3.2.13, build 6, reset 4
GYRO: x= 0.000 y= 0.002 z=-0.004 rad/s acc=0
MAG: x=-20.88 y=-34.13 z=-15.13 uT acc=0
BNO085 DMA stream: RV/GAME/ACC/GYRO=100 Hz, MAG=25 Hz
YPR: yaw=... roll=... pitch=... deg
RATE/s: rv=100 game=100 acc=123 gyro=100 mag=25
DIAG: pkt=... shtp_gap=0 sensor_gap=0 bad=0 cont=0 io=0 dma_to=0 recover=0 reset=1
```

结果：2.625 MHz SPI DMA 下 Product ID 和 5 种报告均正常；RV、Game RV、
Gyroscope 稳定在约 100 Hz，Magnetometer 稳定在约 25 Hz；Accelerometer
虽请求 100 Hz，当前样机实测约 120–128 Hz。DMA 未完成时服务函数立即返回，
观察窗口内未出现 timeout、invalid report 或自动重启。

扩展验证构建把 7 类可选报告临时设为 25 Hz，并在同一块硬件上确认 Getter：

```text
EXT OK: lin=0.03 grav=-6.10 gyro_bias=0.002 mag_bias=0.00 raw=-1168/-2/393
```

随后发布构建已把 `BNO085_EXTENDED_VALIDATION` 恢复为 `0`，避免默认占用额外带宽。
PC 回归测试输出为 `bno085 parser tests: PASS`。Get Feature 实测确认本固件把
Accelerometer 的 10000 us 请求量化为 8000 us（约 125 Hz）。

说明：这是当前硬件组合的验证记录，不代表所有模块、线长和供电条件都能直接使用 2.625 MHz。
