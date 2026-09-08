# Configuration and event API

The project separates three kinds of configuration so that changing a board
does not change the protocol core.

## 1. Hardware mapping

Pins, peripheral handles, DMA resources, and SDK error codes belong to the Port
layer. STM32 applications fill `BNO085_STM32_PortConfig_t`. Other platforms may
implement `bno085_port.h` directly or use the function-table adapter in
`Port/Callbacks`.

## 2. Driver policy

Start from defaults and change only a value justified by the target:

```c
BNO085_Config_t driver;
BNO085_GetDefaultConfig(&driver);
driver.spi_timeout_ms = 50U;
driver.feature_retry_count = 3U;
BNO085_InitWithConfig(&driver);
```

Zero timeouts, zero drain limits, and zero retry counts are rejected. Hardware
addresses and pins are intentionally absent from this structure.

## 3. SH-2 report configuration

```c
BNO085_ReportConfig_t step = {
    .interval_us = 20000U,
    .batch_interval_us = 1000000U,
    .sensor_specific = 0U,
    .change_sensitivity = 0U,
    .flags = BNO085_FEATURE_WAKEUP_ENABLED
};
BNO085_ConfigureReport(BNO085_REPORT_STEP_COUNTER, &step);
```

`interval_us` controls generation, `batch_interval_us` permits SH-2 to buffer
reports, `change_sensitivity` configures report-on-change behavior, and flags
select relative/absolute sensitivity, wake, always-on, and sniff behavior.
Support for individual flag combinations depends on the firmware and report.

Use `BNO085_GetReportConfig()` to read the effective configuration. Firmware may
quantize an interval. Use `BNO085_FlushReport()` to request immediate delivery
of batched samples, and `BNO085_DisableReport()` to set a zero interval.

Wake reports arrive on SHTP channel 4. Both blocking and asynchronous receive
paths parse channels 3 and 4.

## Polling and callbacks

The low-level Poll APIs remain useful for small bare-metal applications.
`BNO085_Process()` adds optional callback dispatch without changing ownership of
the execution context:

```c
static void data_ready(uint32_t events, void *user)
{
    if (events & BNO085_EVENT_ROTATION_VECTOR) {
        BNO085_Euler_t value;
        (void)BNO085_GetEuler(&value);
    }
}

BNO085_Callbacks_t cb = { data_ready, error_handler, application };
BNO085_SetCallbacks(&cb);

for (;;) {
    (void)BNO085_Process(NULL);
}
```

Callbacks run synchronously inside `BNO085_Process()`. They must be short and
must not recursively call Poll or Process. Getter calls are safe because they
only copy cached data.
