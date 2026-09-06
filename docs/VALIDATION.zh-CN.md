# 实机验证记录

验证日期：2026-09-06

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
Program Size: Code=21252 RO-data=524 RW-data=108 ZI-data=2364
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
```

结果：2.625 MHz SPI DMA 下 Product ID 和 5 种报告均正常；RV、Game RV、
Gyroscope 稳定在约 100 Hz，Magnetometer 稳定在约 25 Hz；Accelerometer
虽请求 100 Hz，当前样机实测约 120–128 Hz。DMA 未完成时服务函数立即返回，
观察窗口内未出现 timeout、invalid report 或自动重启。

说明：这是当前硬件组合的验证记录，不代表所有模块、线长和供电条件都能直接使用 2.625 MHz。
