/* SPDX-License-Identifier: BSD-3-Clause */
#include "bno085_port_callbacks.h"

#include <stddef.h>

static BNO085_CallbackPortConfig_t callbacks;
static bool ready;

bool BNO085_CallbackPort_Init(const BNO085_CallbackPortConfig_t *config)
{
    if ((config == NULL) || (config->spi_transfer == NULL) ||
        (config->spi_transfer_async == NULL) ||
        (config->spi_async_status == NULL) ||
        (config->spi_async_abort == NULL) || (config->recover == NULL) ||
        (config->get_last_error == NULL) || (config->set_cs == NULL) ||
        (config->set_wake == NULL) || (config->set_reset == NULL) ||
        (config->interrupt_asserted == NULL) ||
        (config->get_time_ms == NULL) || (config->delay_ms == NULL)) {
        return false;
    }
    callbacks = *config;
    ready = true;
    callbacks.set_cs(callbacks.context, false);
    callbacks.set_wake(callbacks.context, false);
    return true;
}

bool BNO085_Port_IsReady(void) { return ready; }
bool BNO085_Port_SPITransfer(const uint8_t *tx, uint8_t *rx,
                            uint16_t length, uint32_t timeout_ms)
{ return ready && callbacks.spi_transfer(callbacks.context, tx, rx, length,
                                          timeout_ms); }
bool BNO085_Port_SPITransferAsync(const uint8_t *tx, uint8_t *rx,
                                 uint16_t length)
{ return ready && callbacks.spi_transfer_async(callbacks.context, tx, rx,
                                                length); }
BNO085_PortAsyncStatus_t BNO085_Port_SPITransferAsyncStatus(void)
{ return ready ? callbacks.spi_async_status(callbacks.context) :
                 BNO085_PORT_ASYNC_ERROR; }
void BNO085_Port_SPITransferAsyncAbort(void)
{ if (ready) callbacks.spi_async_abort(callbacks.context); }
BNO085_PortError_t BNO085_Port_GetLastError(uint32_t *raw_error)
{ return ready ? callbacks.get_last_error(callbacks.context, raw_error) :
                 BNO085_PORT_ERROR_SPI; }
bool BNO085_Port_Recover(void)
{ return ready && callbacks.recover(callbacks.context); }
void BNO085_Port_SetChipSelect(bool asserted)
{ if (ready) callbacks.set_cs(callbacks.context, asserted); }
void BNO085_Port_SetWake(bool asserted)
{ if (ready) callbacks.set_wake(callbacks.context, asserted); }
void BNO085_Port_SetReset(bool asserted)
{ if (ready) callbacks.set_reset(callbacks.context, asserted); }
bool BNO085_Port_IsInterruptAsserted(void)
{ return ready && callbacks.interrupt_asserted(callbacks.context); }
uint32_t BNO085_Port_GetTimeMs(void)
{ return ready ? callbacks.get_time_ms(callbacks.context) : 0U; }
void BNO085_Port_DelayMs(uint32_t delay_ms)
{ if (ready) callbacks.delay_ms(callbacks.context, delay_ms); }
