# Development process: from silence to a reliable DMA stream

This note records the reasoning used to build and debug the driver. It is meant
to be reusable: diagnose the lowest layer first, and do not change several
protocol assumptions at once.

## 1. Establish electrical facts

Before parsing bytes, verify power, common ground, interface-selection straps,
and the four host-control signals. For SPI mode in this project, PS0/WAKE and PS1
are sampled high when reset is released, BOOTN is high, `H_INTN` is an input,
and chip select is driven by software.

The first useful test is not yaw. It is whether `H_INTN` becomes low after reset.
If it never does, inspecting SH-2 report IDs is premature.

## 2. Prove one complete SHTP transaction

When `H_INTN` is low:

1. Pull CS low.
2. Clock four bytes to receive the SHTP header.
3. Decode the little-endian packet length.
4. Continue clocking the remaining payload without releasing CS.
5. Pull CS high only after the packet is fully drained.

The BNO085 SPI receive path must transmit `0x00` dummy bytes. Using `0xFF` was a
critical early failure: the device could boot and identify itself, yet later
traffic became unreliable.

## 3. Treat startup as a protocol state machine

The sensor sends an SHTP advertisement first, followed by SH-2 initialization
events. An advertisement only proves the transport is alive. Report
configuration starts only after the executable channel reports reset complete.

The working order is:

```text
hardware reset
  -> drain advertisement
  -> wait for SH-2 reset complete
  -> request Product ID
  -> configure calibration
  -> Set Feature
  -> Get Feature and verify the effective interval
  -> receive sensor reports
```

Sending Set Feature immediately after advertisement caused the command to be
silently ignored on the tested firmware.

## 4. Add observability before adding features

Every layer receives its own evidence:

- Product ID proves the control channel can round-trip.
- SHTP sequence gaps indicate transport loss.
- Sensor report sequence gaps indicate report-level loss.
- Invalid-packet and continuation counters expose parser assumptions.
- Port error and DMA-timeout counters expose hardware transport failures.
- Requested and effective report intervals expose firmware quantization.

This distinction prevented an 8 ms accelerometer interval from being mistaken
for packet loss after a 10 ms request.

## 5. Verify commands, not just writes

SH-2 Set Feature does not provide a dedicated acknowledgement packet. The driver
therefore follows it with Get Feature and matches the returned feature ID. It
accepts reasonable firmware interval quantization and still requires actual
sensor data before declaring the stream healthy.

Commands that do have responses—calibration, DCD save, and tare—are matched by
command ID and command sequence, with bounded timeouts.

## 6. Optimize only after the blocking path works

The initial implementation used blocking SPI because it was easy to observe.
Once framing and startup were correct, streaming was converted to two-stage DMA:
header DMA, validate length, then payload DMA while CS remains low.

The asynchronous API advances one small state-machine step and returns. It does
not busy-wait and it does not force the application to sleep. This keeps policy
in the application and mechanism in the driver.

## 7. Recover at the cheapest useful level

The final design uses escalating recovery:

- A malformed packet or isolated sequence gap is counted and discarded.
- Repeated transport errors release CS and rebuild SPI/DMA state.
- A sustained absence of Rotation Vector data performs a full BNO085 restart.

The medium recovery path preserves the configured SH-2 report state, avoiding a
sensor reboot for a recoverable MCU peripheral fault.

## 8. Expand reports through a common parser

Each new report follows the same pattern: define its ID and Q point, verify its
wire length, decode little-endian fields, update a typed cache, set an event bit,
and test the parser with a synthetic packet. This added linear acceleration,
gravity, uncalibrated sensors, and raw reports without duplicating transport
logic.

## 9. Verify on both host and hardware

Host tests make protocol corner cases deterministic. Hardware testing proves
electrical timing, HAL/DMA behavior, real firmware interval choices, and
long-running stream health. Neither replaces the other.

The final validation checked a warning-free Keil build, host parser tests, all
optional report types, measured per-second rates, and zero transport/parser
errors during the observation window.
