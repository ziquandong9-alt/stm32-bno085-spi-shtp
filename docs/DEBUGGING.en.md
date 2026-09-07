# Troubleshooting BNO085 SPI/SHTP

Use the serial symptom to choose a layer. Resetting the sensor after every error
hides the distinction between wiring, framing, startup order, and report parsing.

## No output at all

Check the MCU image, UART pins and baud rate, power, ground, and whether the
application reaches its boot message. This is not yet a BNO085 protocol problem.

## `H_INTN` never goes low after reset

Check NRST pulse timing, PS0/PS1 interface selection, BOOTN, signal voltage, and
the `H_INTN` pull-up. Confirm that `H_INTN` is configured as an input and CS is
high while idle.

## Product ID works, but enabling reports times out

This strongly suggests that SPI electrical communication works but the SH-2
startup order is wrong. Drain the advertisement, then wait for reset complete on
the executable channel before sending control commands.

Also verify channel-specific sequence numbers and transmit `0x00` dummy bytes
during SPI reads. The latter was essential on the tested hardware.

## `enable failed: timeout`

Set Feature has no standalone ACK. The driver verifies it with Get Feature and
matches Feature Response by feature ID. Inspect diagnostics for:

- `feature_verify_fail`: response missing or inconsistent;
- `io`/`dma_to`: lower-level transport problem;
- `bad`/`cont`: parser or unsupported fragmentation;
- effective interval: valid firmware quantization rather than failure.

After verification, actual arrival of the requested report remains the final
proof that the stream is active.

## Product ID repeats endlessly

The application is entering its full restart path. Look at the error printed
immediately before each Product ID. Repeated transport errors should first use
the medium SPI/DMA recovery; repeated absence of Rotation Vector for one second
then escalates to sensor reset and reconfiguration.

## YPR streams but accuracy stays 0

Accuracy is a 0–3 confidence/calibration status, not degrees. Keep the sensor
still briefly for gyro calibration, rotate it through varied orientations for
accelerometer calibration, and make slow figure-eight motions away from motors,
magnets, high-current wiring, and large steel objects for magnetometer
calibration.

An accelerometer status of 2 with magnetometer status 0 can still produce YPR,
but absolute yaw is not trustworthy. Use Game Rotation Vector if stable relative
orientation matters more than north-referenced heading.

## Rates differ from the request

Read the effective interval returned by Get Feature. SH-2 may quantize a requested
period; the tested device maps 10 ms accelerometer reporting to 8 ms. A measured
125 Hz stream in that case is correct and should not increment sequence-gap
counters.

## Values jump or reports become invalid

Verify that CS remains low for header plus payload, packet length includes the
four-byte header, bit 15 is treated as continuation, and all multi-byte fields
are little-endian. Drain the whole packet even if the local parser cannot store
or understand its payload, otherwise the next header starts at the wrong byte.

## DMA stalls with CS low

Inspect `dma_to`, `io`, and the raw port error. The STM32 port's error callback
must mark the transfer failed, release CS, abort/deinitialize/reinitialize SPI
and DMA, and return the asynchronous state to idle. A CS-low watchdog prevents a
failed callback path from locking the bus permanently.

## Yaw/roll/pitch axes look wrong

The example uses Z-Y-X Euler angles: yaw about Z, pitch about Y, roll about X.
Board mounting orientation may differ from the desired product frame. Preserve
the quaternion and apply a documented mounting transform instead of swapping
Euler outputs until one pose appears correct.

## A new report never appears

Confirm its report ID, minimum wire length, Q point, feature response, and event
bit. Temporarily enable only that report to reduce traffic. Some long or special
reports may require full SHTP fragment reassembly, which this version does not
yet provide; `cont` makes that limitation visible instead of silently parsing
partial data.

## Recommended capture for an issue

Include the MCU, BNO085 firmware line, SPI clock/mode, enabled reports and
intervals, several seconds of `RATE/s` and `DIAG` output, and the exact failure
message. Do not publish debugger serial numbers, local user paths, private board
names, or unrelated application code.
