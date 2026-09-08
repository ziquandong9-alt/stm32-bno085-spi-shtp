/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef BNO085_PORT_CALLBACKS_H
#define BNO085_PORT_CALLBACKS_H

#include "bno085_port.h"

/** SDK-neutral function table. Compile bno085_port_callbacks.c instead of the
 * STM32 port to integrate ESP-IDF, Pico SDK, GD32 SPL/HAL, Zephyr, or an RTOS.
 * 与 SDK 无关的函数表；可替代 STM32 Port 接入任意平台。 */
typedef struct {
    void *context;
    bool (*spi_transfer)(void *context, const uint8_t *tx, uint8_t *rx,
                         uint16_t length, uint32_t timeout_ms);
    bool (*spi_transfer_async)(void *context, const uint8_t *tx, uint8_t *rx,
                               uint16_t length);
    BNO085_PortAsyncStatus_t (*spi_async_status)(void *context);
    void (*spi_async_abort)(void *context);
    bool (*recover)(void *context);
    BNO085_PortError_t (*get_last_error)(void *context, uint32_t *raw_error);
    void (*set_cs)(void *context, bool asserted);
    void (*set_wake)(void *context, bool asserted);
    void (*set_reset)(void *context, bool asserted);
    bool (*interrupt_asserted)(void *context);
    uint32_t (*get_time_ms)(void *context);
    void (*delay_ms)(void *context, uint32_t delay_ms);
} BNO085_CallbackPortConfig_t;

bool BNO085_CallbackPort_Init(const BNO085_CallbackPortConfig_t *config);

#endif
