# Porting the driver to another MCU

Keep the protocol files unchanged:

```text
BNO085_Driver/Inc/bno085.h
BNO085_Driver/Src/bno085.c
BNO085_Driver/Port/bno085_port.h
```

Replace only the implementation under `BNO085_Driver/Port/STM32`, or add a new
sibling directory for your SDK.

For the shortest integration, compile `Port/Callbacks/bno085_port_callbacks.c`
and fill `BNO085_CallbackPortConfig_t` with SDK functions. For maximum control
and minimum call overhead, copy `Port/Template/bno085_port_template.c` into a
new platform directory and implement the contract directly. The template has a
deliberate `#error`, so an unfinished port cannot be shipped accidentally.

## Required platform services

Implement every function declared in `bno085_port.h`:

- initialize the port configuration;
- drive CS, WAKE, and RESET;
- read active-low `H_INTN`;
- provide blocking SPI transmit/receive for startup and control commands;
- start asynchronous full-duplex SPI transfer;
- report asynchronous busy/complete/error state;
- provide monotonic time and bounded delay;
- abort and rebuild the bus after an error;
- return a raw platform error value for diagnostics.

Map SDK-specific results to the normalized BNO085 port errors. Do not leak HAL
enums into the protocol core.

## SPI requirements

- Mode 3: CPOL high, sample on the second edge
- MSB first
- Clock no higher than the BNO085 limit
- Software-controlled CS
- `0x00` dummy bytes while receiving
- One continuous CS-low interval for the SHTP header and its payload

The asynchronous implementation must preserve that last rule. A header DMA and
payload DMA are two hardware operations but one SPI transaction from the
sensor's point of view.

## GPIO and boot requirements

- CS idles high.
- NRST is actively driven low and then high.
- WAKE/PS0 remains high for normal SPI communication.
- PS1 is sampled high during reset release.
- BOOTN is high for normal application boot.
- `H_INTN` is a readable active-low input; an external interrupt is optional.

Polling `H_INTN` is sufficient because `BNO085_PollAsync()` returns immediately.
An EXTI handler may set a flag or wake a task, but should not run the complete
parser inside interrupt context.

## Timebase requirements

Use a monotonic clock whose wraparound behavior is compatible with unsigned
subtraction. Millisecond timing controls startup and recovery timeouts;
microsecond timing anchors report timestamps and performance diagnostics.

If the MCU lacks a native microsecond clock, extend a hardware timer. Avoid a
wall clock that can jump after synchronization.

## DMA integration

The port owns SDK/DMA callbacks. On completion it records success; on error it
records a normalized error and the raw SDK code. The core observes that state on
the next `BNO085_PollAsync()` call.

Recovery must always:

1. release CS;
2. abort the active transfer;
3. clear DMA/SPI error state;
4. reinitialize the peripheral and linked DMA resources;
5. return the async state to idle.

Do not assume the normal completion callback runs after every failed transfer.

## Recommended bring-up order

1. Scope reset, CS, clock, MOSI, MISO, and `H_INTN`.
2. Read and validate one advertisement packet with blocking SPI.
3. Wait for reset complete.
4. Read Product ID.
5. Enable one low-rate Rotation Vector report and verify Get Feature.
6. Confirm stable blocking reception.
7. Enable asynchronous/DMA reception.
8. Add other reports one at a time and monitor diagnostics.
9. Inject an SPI/DMA error and verify CS is released and the stream recovers.

## Port acceptance checklist

- Warning-free target build
- Product ID is stable over repeated resets
- No SHTP or sensor sequence gaps at the intended report rates
- No CS-low hang after an injected transport error
- Requested/effective intervals are reported correctly
- Host parser tests still pass without platform headers
- At least several minutes of streaming with `bad=0`, `io=0`, and `dma_to=0`
