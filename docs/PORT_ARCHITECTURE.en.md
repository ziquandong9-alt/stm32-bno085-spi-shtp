# Port architecture and supported environments

| Component | Status | Notes |
|---|---|---|
| Protocol core | Supported | ISO C99, no STM32 headers |
| STM32F4 HAL SPI/DMA | Hardware validated | Included example project |
| Callback-table port | Supported | SDK-neutral integration boundary |
| Port template | Supported | Compile-time TODO guard prevents accidental use |
| Keil MDK | Hardware validated | STM32F407ZG target |
| CMake + GCC/Clang | CI/host tested | Library, tests, install/export |
| STM32CubeIDE | Import path documented | Generate/import from the included `.ioc` |
| FreeRTOS | Reference example | Task notification and overwrite queue |
| I2C/SHTP | Not implemented | Requires transport-specific packet limits/timing |
| UART-RVC | Not implemented | Different fixed-frame API; should be a separate backend |
| Multi-instance | Planned breaking change | Requires all static state to move into a handle |

For a new MCU, prefer the callback-table port when the SDK can expose the
required full-duplex async transfer. Use a dedicated compiled port when direct
IRQ/DMA integration or minimum overhead matters.

Never emulate the async callback with a long blocking SPI operation in an ISR.
If the SDK has no async SPI facility, keep the blocking `BNO085_Poll()` path or
add a worker task in the port.
