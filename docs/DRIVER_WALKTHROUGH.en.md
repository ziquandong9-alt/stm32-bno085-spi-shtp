# BNO085 SPI/SHTP driver walkthrough

This guide follows the code in execution order. Keep the datasheet, SHTP manual,
and SH-2 reference manual nearby: the electrical interface is SPI, but the data
model is packet- and report-oriented.

## 1. Start at the public API

`BNO085_Driver/Inc/bno085.h` is the application contract. It contains public
data types, event bits, error codes, configuration functions, polling functions,
and cached getters. Helper functions and private protocol constants stay in
`bno085.c` as `static` symbols.

The driver is currently single-instance, so state lives in one private context.
That deliberate constraint keeps the API small; it can later become a handle
without changing packet parsing concepts.

## 2. Understand the port boundary

`BNO085_Driver/Port/bno085_port.h` defines everything the protocol core may ask
the platform to do:

- assert and release CS;
- read `H_INTN`;
- drive WAKE and RESET;
- perform blocking SPI transfer;
- start and inspect an asynchronous SPI transfer;
- read a monotonic microsecond/millisecond clock;
- recover the bus after a transport failure.

Only `Port/STM32` calls STM32 HAL. A new MCU port implements this contract rather
than editing `bno085.c`.

## 3. Initialize GPIO before resetting the sensor

The STM32 port stores the SPI handle and GPIO mapping, drives CS high, keeps WAKE
high for SPI operation, and starts from an idle asynchronous state. Reset is
then pulsed low and released. The driver waits for `H_INTN`, rather than assuming
a fixed boot time alone is sufficient.

## 4. Read an SHTP packet correctly

Every packet starts with:

```text
byte 0..1  little-endian length, including the four-byte header
byte 2     channel
byte 3     channel sequence number
```

Bit 15 of the length indicates continuation. The implementation masks that bit
before checking the size. It detects and counts continuation packets, and safely
rejects them on the common streaming path; arbitrary fragment reassembly is not
yet implemented.

CS must stay low between header and payload. Releasing it after four bytes would
turn the payload into a new transaction and desynchronize the device stream.

## 5. Track channel sequence numbers independently

SHTP sequence numbers are scoped to a channel, not to the whole device. The
driver therefore keeps separate TX and RX state per channel. A gap increments a
diagnostic counter, while the current packet can still be inspected if its
framing is valid.

Sensor reports also contain their own sequence values. Tracking those separately
answers two different questions: did the SPI/SHTP transport lose a packet, or
did a particular SH-2 report stream skip a sample?

## 6. Wait for reset complete

`BNO085_Init()` drains startup traffic until the SH-2 reset-complete indication
arrives on the executable channel. The advertisement is parsed as transport
setup, not as permission to configure reports.

Only then does the application request Product ID and print part, firmware,
build, and reset-reason information.

## 7. Configure calibration intentionally

`BNO085_SetCalibration()` builds the SH-2 Configure ME command and waits for the
matching command response. The example enables accelerometer, gyroscope,
magnetometer, and on-table calibration. Calibration status is metadata; a zero
status does not mean the quaternion itself is zero.

`BNO085_SaveCalibration()` persists dynamic calibration data inside the sensor.
Because this writes nonvolatile memory, call it only after calibration converges
and only when the application explicitly chooses to save.

## 8. Enable a report and read back the result

An enable function sends Set Feature (`0xFD`) on the control channel with the
report ID and requested interval. The driver then sends Get Feature (`0xFE`) and
waits for Feature Response (`0xFC`) with the same feature ID.

The effective interval can differ from the request because SH-2 selects a period
supported by its algorithm. The tested firmware returns 8000 us for a 10000 us
accelerometer request. This is recorded in diagnostics and accepted within a
bounded tolerance.

## 9. Advance the non-blocking receive state machine

`BNO085_PollAsync()` has no wait loop. Conceptually it performs:

```text
IDLE
  H_INTN high -> return PENDING
  H_INTN low  -> assert CS, start 4-byte header DMA

HEADER_DMA
  busy        -> return PENDING
  complete    -> validate length, start payload DMA

PAYLOAD_DMA
  busy        -> return PENDING
  complete    -> release CS, parse all reports, return events
```

A watchdog bounds how long CS may remain low. HAL SPI/DMA errors are normalized
by the port, CS is released, and the application can escalate recovery.

## 10. Parse the Base Timestamp container

Sensor data usually arrives on channel 3 inside a Base Timestamp report. The
container establishes a packet time reference and may hold several sensor
reports. The parser walks the cargo once, determines each report length, and
dispatches by report ID.

Timestamp Rebase changes the reference for following reports. Each report also
has a 14-bit delay split between status bits and a delay byte. The driver applies
both elements to the MCU-captured packet time to estimate `timestamp_us`.

## 11. Decode fixed-point values

SH-2 values use signed fixed-point formats. The conversion is:

```text
physical_value = signed_integer / 2^Q
```

Rotation Vector quaternion components use Q14 and its accuracy estimate uses
Q12. Calibrated acceleration uses Q8, gyroscope Q9, and magnetic field Q4. Always
decode the little-endian integer as signed before scaling.

## 12. Convert the quaternion once

When a new Rotation Vector arrives, the driver caches the quaternion and derives
Z-Y-X Euler angles once. It clamps the pitch input to the valid inverse-sine
domain to protect against rounding near ±90 degrees.

The public getters then return one coherent cached sample:

```c
float yaw   = BNO085_GetYaw();
float roll  = BNO085_GetRoll();
float pitch = BNO085_GetPitch();
```

They do not touch SPI. The same rule applies to acceleration, angular velocity,
magnetic field, linear acceleration, gravity, uncalibrated, and raw getters.

## 13. Consume event bits in the application

One payload can update several caches, so `BNO085_PollAsync()` returns a bitmask
rather than a single report type. `bno085_process()` handles the bits, prints at
a throttled rate, updates per-second counters, and leaves unrelated application
work outside the driver.

Do not block the 100 Hz receive path with excessive UART printing. For a control
application, use UART DMA or print only periodic diagnostics.

## 14. Read diagnostics before resetting everything

`BNO085_GetDiagnostics()` exposes packet counts, SHTP/report gaps, invalid and
continuation packets, port errors, DMA timeouts, feature verification failures,
recoveries, resets, and the last raw platform error.

The example first discards isolated bad traffic, rebuilds SPI/DMA after repeated
transport errors, and performs a complete BNO085 restart only after sustained
loss of Rotation Vector data.

## 15. Add another SH-2 report

To add a report safely:

1. Add its public data structure, event bit, enable API, and getter.
2. Add the private report ID, expected wire length, and Q point.
3. Reuse the common Set/Get Feature path.
4. Decode it in the sensor-report dispatcher and update its cache atomically.
5. Add a synthetic packet to `tests/test_bno085_parser.c`.
6. Enable it temporarily on hardware and compare its measured rate with the
   effective feature interval.

This workflow extends the sensor layer without modifying SHTP framing or the
platform port.
