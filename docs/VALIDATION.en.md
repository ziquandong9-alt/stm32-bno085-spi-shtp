# Hardware validation record

This record summarizes the reproducible checks performed on the publication
candidate. It contains no debugger serial number, workstation path, or private
device identifier.

## Target

- STM32F407ZGTx
- BNO085 part `10004148`, firmware `3.2.13`, build `6`
- SPI mode 3 at 2.625 Mbit/s
- USART1 at 115200 8-N-1
- Keil MDK / ARM Compiler 5.06 update 7

## Build and host test

- Target build: 0 errors, 0 warnings
- Image size: Code 26772, RO-data 728, RW-data 184, ZI-data 3160 bytes
- Host parser/fault-injection test: PASS with `-Wall -Wextra -Werror`

## Streaming observations

The normal configuration produced approximately:

| Report | Observed rate |
|---|---:|
| Rotation Vector | 100–103 Hz |
| Game Rotation Vector | 100–103 Hz |
| Accelerometer | 122–130 Hz |
| Gyroscope | 101–102 Hz |
| Magnetometer | 25–26 Hz |

The accelerometer difference is explained by Feature Response: the firmware
selected an 8000 us effective interval for a 10000 us request.

During the final validation window (more than 8,600 received packets),
diagnostics reported zero SHTP sequence gaps,
zero sensor-report gaps, zero invalid packets, zero continuation packets, zero
port errors, and zero DMA timeouts in the normal report configuration.

## Extended report validation

The optional validation configuration received all enabled report families:

- linear acceleration;
- gravity;
- uncalibrated gyroscope and bias;
- uncalibrated magnetometer and bias;
- raw accelerometer, gyroscope, and magnetometer.

It printed an `EXT OK` marker only after every required event had arrived.

## Recovery coverage

Host fault injection checks the medium transport-recovery path. On target, the
port releases CS on error/timeout and rebuilds SPI/DMA before the application
escalates to a full sensor restart if Rotation Vector remains absent.

These measurements characterize one board and firmware build; they are evidence
for the tested setup, not a guaranteed rate or calibration result for every
BNO085 module.
