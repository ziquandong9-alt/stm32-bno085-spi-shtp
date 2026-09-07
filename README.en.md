# BNO085 SPI/SHTP driver for STM32

[简体中文](README.md) | **English**

An experimentally validated BNO085 SPI/SHTP driver for STM32. The example
targets an STM32F407ZG and uses STM32 HAL and Keil MDK-ARM, while the protocol
core itself has no direct STM32 HAL dependency. A small port layer owns SPI,
DMA, GPIO, and timing, so moving the driver to another MCU does not require
rewriting the SHTP/SH-2 logic.

The streaming path is non-blocking: SPI1 DMA reads Rotation Vector, Game
Rotation Vector, calibrated accelerometer, gyroscope, and magnetometer reports,
while USART1 prints yaw, roll, and pitch continuously.

> BNO085 is not a register-oriented SPI peripheral. Its host interface uses
> SHTP and SH-2, so a working driver must honor `H_INTN`, keep chip select low
> across an entire packet, track channel sequence numbers, and wait for the
> SH-2 reset-complete event before configuring reports.

## Hardware-validated configuration

- MCU: STM32F407ZGTx
- Sensor: BNO085, part `10004148`, firmware `3.2.13`, build `6`
- Toolchain: Keil MDK, ARM Compiler 5.06 update 7
- SPI: mode 3, MSB first, 2.625 Mbit/s
- UART log: USART1, 115200 8-N-1
- Streaming: DMA2 Stream 0/3 with a non-blocking application state machine
- Measured rates: RV/Game RV/Gyroscope about 100 Hz, Magnetometer about 25 Hz;
  the tested firmware quantizes a 10 ms accelerometer request to 8 ms, or about
  125 Hz

Example output:

```text
BNO085 demo boot (SPI DMA)
BNO085 part 10004148, FW 3.2.13, build 6, reset 4
BNO085 DMA stream: RV/GAME/ACC/GYRO=100 Hz, MAG=25 Hz
YPR: yaw= -55.93 roll=-153.71 pitch= -49.01 deg
RATE/s: rv=100 game=100 acc=123 gyro=100 mag=25
DIAG: packet=... shtp_gap=0 sensor_gap=0 bad=0 cont=0 io=0 dma_to=0
```

## Highlights

- Correct BNO085 SPI reset and SH-2 startup sequence
- SHTP framing and independent TX/RX sequence tracking per channel
- Set Feature followed by Get Feature readback and interval verification
- Product ID request and multi-response handling
- Rotation Vector and Game Rotation Vector quaternion decoding
- Cached Z-Y-X yaw, pitch, and roll conversion
- Calibrated acceleration, angular velocity, and magnetic field reports
- Linear acceleration, gravity, uncalibrated gyro/magnetometer, and raw sensor
  reports
- Base Timestamp, Timestamp Rebase, and 14-bit report-delay processing
- Non-blocking two-stage DMA receive: four-byte header followed by payload while
  chip select remains asserted
- Normalized port errors, DMA timeout protection, bus recovery, and full sensor
  restart
- Runtime calibration control, DCD save, and tare commands with response
  matching
- Host-side parser/fault-injection tests and GitHub Actions CI
- Bilingual comments in the driver-facing code

## Repository layout

```text
BNO085_Driver/
├─ Inc/bno085.h              Public application API
├─ Src/bno085.c              MCU-independent SHTP/SH-2 core
└─ Port/
   ├─ bno085_port.h          Hardware abstraction contract
   └─ STM32/                 STM32 HAL implementation
Core/                        CubeMX-generated code and example application
Drivers/                     STM32 HAL and CMSIS
MDK-ARM/BNO085.uvprojx       Keil project
docs/                        English and Simplified Chinese guides
tests/test_bno085_parser.c   Host-side protocol regression tests
```

## Wiring

| BNO085 signal | STM32F407ZG | Purpose |
|---|---:|---|
| H_INTN | PA0 | Active-low data-ready/handshake input |
| NRST | PA1 | Active-low hardware reset |
| WAKE/PS0 | PA2 | Host wake; held high during interface selection |
| CS | PA4 | Software-controlled chip select |
| SCK | PA5 | SPI1 clock |
| MISO | PA6 | SPI1 MISO |
| MOSI | PA7 | SPI1 MOSI |
| UART TX | PA9 | Debug output |
| GND | GND | Common ground |

PS1 must also be high when reset is released to select SPI mode, and BOOTN must
remain high for normal boot. Modules often provide these pull-ups; custom boards
must verify them explicitly.

## Build and run

1. Open `MDK-ARM/BNO085.uvprojx` in Keil.
2. Select the `BNO085` target, build, and program the MCU.
3. Open the serial port at 115200 8-N-1.
4. After Product ID is printed, yaw, roll, and pitch should stream continuously.

Minimal application flow, with error handling omitted:

```c
BNO085_STM32_PortConfig_t port;
uint32_t events;
BNO085_Euler_t euler;

port.spi = &hspi1;
/* Fill the CS, WAKE, RESET, and INT GPIO ports and pins. */
BNO085_STM32_Port_Init(&port);
BNO085_Init();

BNO085_EnableAccelerometer(10000U);       /* Request 100 Hz. */
BNO085_EnableGyroscope(10000U);           /* Request 100 Hz. */
BNO085_EnableMagnetometer(40000U);        /* Request 25 Hz. */
BNO085_EnableGameRotationVector(10000U);  /* Request 100 Hz. */
BNO085_EnableRotationVector(10000U);      /* Request 100 Hz. */

if ((BNO085_PollAsync(&events) == BNO085_OK) &&
    ((events & BNO085_EVENT_ROTATION_VECTOR) != 0U)) {
    BNO085_GetEuler(&euler);
}
```

`BNO085_PollAsync()` starts or advances a DMA transaction and returns
`BNO085_PENDING` immediately while hardware is busy. Getter functions only read
the latest cached frame; calling yaw, roll, and pitch getters does not cause
three SPI transactions.

The example keeps all streaming work inside `bno085_process()`. The main loop
chooses whether to perform other work, enter a low-power state, or yield to an
RTOS. The driver does not impose `WFI`.

## Calibration and timestamps

The accuracy field is a status value from 0 to 3, not an angle in degrees. A
Rotation Vector with accuracy 0 still contains usable quaternion data, but its
absolute heading is not yet trustworthy. Game Rotation Vector avoids magnetic
disturbance at the cost of long-term yaw drift.

Call `BNO085_SaveCalibration()` only after calibration has stabilized; it writes
the BNO085 nonvolatile storage and must not be called continuously. Mounting
offsets can be managed with `BNO085_TareNow()`, `BNO085_PersistTare()`, and
`BNO085_ClearTare()`.

The public `timestamp_us` starts from the MCU packet-arrival time and applies
SH-2 Base Timestamp, Timestamp Rebase, and report delay. Raw reports also expose
the sensor-provided `sensor_timestamp_us`.

## SPI versus UART-RVC

UART-RVC runs at 115200 bit/s and outputs a fixed 19-byte frame at 100 Hz. The
example SPI clock is 2.625 Mbit/s, about 22.8 times faster at the physical layer.
A UART-RVC frame occupies roughly 1.65 ms on the wire, whereas a typical 23-byte
SHTP Rotation Vector packet takes about 70 us of SPI shift time.

This does **not** make a 100 Hz attitude estimate update 23 times faster. SPI's
advantages are lower bus occupancy, lower host read latency, selectable report
types, and concurrent access to many SH-2 outputs. UART-RVC is preferable when a
minimal fixed-format YPR stream is all that is required.

## Host tests

```sh
gcc -std=c11 -Wall -Wextra -Werror \
  -IBNO085_Driver/Inc -IBNO085_Driver/Port \
  tests/test_bno085_parser.c -lm -o test_bno085_parser
./test_bno085_parser
```

The test covers timestamp rebasing, fixed-point decoding, multiple reports in
one payload, report sequence gaps, and injected SPI/DMA recovery faults. CI runs
it on every push and pull request.

## Learning path

1. [Development process](docs/DEVELOPMENT_PROCESS.en.md) — how the failure was
   separated into electrical, transport, protocol, and report-layer problems.
2. [Driver walkthrough](docs/DRIVER_WALKTHROUGH.en.md) — follow execution from
   reset and SPI framing to the cached yaw/pitch/roll API.
3. [Troubleshooting guide](docs/DEBUGGING.en.md) — map serial symptoms to the
   layer most likely responsible.
4. [Porting guide](docs/PORTING.en.md) — implement the hardware abstraction for
   another MCU or SDK.
5. [Hardware validation](docs/VALIDATION.en.md) — reproducible build and
   on-board test evidence.

## Current scope

The driver intentionally remains single-instance. Full generic reassembly of
arbitrarily fragmented SHTP payloads and MCU-specific DMA double-buffer mode are
not implemented; continuation packets are detected, counted, and rejected
safely. These are better added when a real enabled report requires them rather
than complicating the verified common path.

## References

- [CEVA BNO08X Datasheet](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)
- [CEVA Sensor Hub Transport Protocol](https://www.ceva-ip.com/wp-content/uploads/Sensor-Hub-Transport-Protocol.pdf)
- [CEVA SH-2 Reference Manual](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf)
- [CEVA SH-2 reference implementation](https://github.com/ceva-dsp/sh2)
- [CEVA STM32/Nucleo demo](https://github.com/ceva-dsp/sh2-demo-nucleo)

## License

Original driver and example changes are licensed under BSD-3-Clause. STM32 HAL,
CMSIS, and other third-party components retain the licenses and copyright notices
in their respective directories.
