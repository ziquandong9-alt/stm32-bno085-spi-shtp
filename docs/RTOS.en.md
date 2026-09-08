# RTOS integration

The driver does not call an RTOS API and does not create a task. This keeps the
core usable with FreeRTOS, ThreadX, Zephyr, or bare metal.

The recommended ownership model is one sensor task per driver instance:

1. `H_INTN` EXTI notifies the sensor task.
2. The task calls `BNO085_Process()` until it returns `BNO085_PENDING`.
3. The data callback copies a cached snapshot to a queue of length one.
4. Control tasks consume the latest snapshot without owning SPI.
5. A short task timeout also services DMA completion if the port does not send
   a separate task notification.

See `examples/freertos`. Do not parse SHTP or invoke application callbacks in
the EXTI ISR. Do not let several tasks call Poll/Process concurrently. Commands
such as Set Feature, calibration save, and tare should be serialized through the
same sensor task.

For low power, configure selected reports with
`BNO085_FEATURE_WAKEUP_ENABLED`, use a nonzero batch interval when latency permits,
and let the board/RTOS idle policy sleep the MCU. The core never executes `WFI`.
