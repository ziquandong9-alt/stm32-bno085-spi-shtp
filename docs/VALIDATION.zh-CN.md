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
Program Size: Code=15756 RO-data=448 RW-data=52 ZI-data=2084
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
BNO085 part 10004148, FW 3.2.13, build 6, reset 4
ACC: x=  8.01 y= -2.84 z= -5.02 m/s^2
BNO085 rotation vector and accelerometer running at 100 Hz
YPR: yaw=... roll=... pitch=... deg
```

结果：2.625 MHz SPI 下 Product ID、加速度和 Rotation Vector 均正常；YPR 持续约 100 Hz
输出，观察窗口内未出现 timeout、invalid report 或自动重启。

说明：这是当前硬件组合的验证记录，不代表所有模块、线长和供电条件都能直接使用 2.625 MHz。
