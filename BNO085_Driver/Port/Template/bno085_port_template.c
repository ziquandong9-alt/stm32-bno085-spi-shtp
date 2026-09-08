/* SPDX-License-Identifier: BSD-3-Clause */
/**
 * Copy this file into a platform-specific directory and replace every TODO.
 * 复制到新平台目录并实现所有 TODO；不要修改协议核心 bno085.c。
 */
#include "bno085_port.h"

#error "Port template only: implement the functions for your MCU/SDK"

bool BNO085_Port_IsReady(void) { return false; }
bool BNO085_Port_SPITransfer(const uint8_t *tx, uint8_t *rx,
                            uint16_t length, uint32_t timeout_ms)
{ (void)tx; (void)rx; (void)length; (void)timeout_ms; return false; }
bool BNO085_Port_SPITransferAsync(const uint8_t *tx, uint8_t *rx,
                                 uint16_t length)
{ (void)tx; (void)rx; (void)length; return false; }
BNO085_PortAsyncStatus_t BNO085_Port_SPITransferAsyncStatus(void)
{ return BNO085_PORT_ASYNC_ERROR; }
void BNO085_Port_SPITransferAsyncAbort(void) { }
BNO085_PortError_t BNO085_Port_GetLastError(uint32_t *raw_error)
{ if (raw_error != NULL) *raw_error = 0U; return BNO085_PORT_ERROR_NONE; }
bool BNO085_Port_Recover(void) { return false; }
void BNO085_Port_SetChipSelect(bool asserted) { (void)asserted; }
void BNO085_Port_SetWake(bool asserted) { (void)asserted; }
void BNO085_Port_SetReset(bool asserted) { (void)asserted; }
bool BNO085_Port_IsInterruptAsserted(void) { return false; }
uint32_t BNO085_Port_GetTimeMs(void) { return 0U; }
void BNO085_Port_DelayMs(uint32_t delay_ms) { (void)delay_ms; }
