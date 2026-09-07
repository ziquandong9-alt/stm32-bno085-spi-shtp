/* SPDX-License-Identifier: BSD-3-Clause */
/**
 * @file bno085_port.h
 * @brief MCU-independent hardware contract for the BNO085 core.
 *        BNO085 核心驱动所依赖的、与 MCU 无关的硬件接口契约。
 *
 * Porting rule / 移植原则:
 * Keep bno085.c unchanged and implement these functions for the new MCU.
 * 保持 bno085.c 不变，只为新 MCU 实现下面这些函数。
 */
#ifndef BNO085_PORT_H
#define BNO085_PORT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * All control signals are active low: asserted=true must drive the pin low.
 * 所有控制信号均低有效：asserted=true 时必须将引脚拉低。
 */
/** Return whether the board-specific adapter was configured. / 平台层是否已配置。 */
bool BNO085_Port_IsReady(void);

/**
 * Perform one blocking full-duplex SPI transfer while CS is already active.
 * 在 CS 已由核心拉低期间，执行一次阻塞式全双工 SPI 传输。
 * The port must not toggle CS inside this function. / 本函数内部不得切换 CS。
 */
bool BNO085_Port_SPITransfer(const uint8_t *tx, uint8_t *rx,
                            uint16_t length, uint32_t timeout_ms);

/** State of one interrupt/DMA SPI transfer. / 一次中断或 DMA SPI 传输的状态。 */
typedef enum {
    BNO085_PORT_ASYNC_IDLE = 0,
    BNO085_PORT_ASYNC_BUSY,
    BNO085_PORT_ASYNC_COMPLETE,
    BNO085_PORT_ASYNC_ERROR
} BNO085_PortAsyncStatus_t;

/** Normalized platform I/O failure. / 与芯片厂商无关的平台 I/O 错误。 */
typedef enum {
    BNO085_PORT_ERROR_NONE = 0,
    BNO085_PORT_ERROR_BUSY,
    BNO085_PORT_ERROR_TIMEOUT,
    BNO085_PORT_ERROR_SPI,
    BNO085_PORT_ERROR_DMA
} BNO085_PortError_t;

/**
 * Start one non-blocking full-duplex transfer while CS is already active.
 * 在 CS 已拉低时启动一次非阻塞全双工传输。
 */
bool BNO085_Port_SPITransferAsync(const uint8_t *tx, uint8_t *rx,
                                 uint16_t length);

/** Query the transfer without waiting. / 查询异步传输状态，不等待。 */
BNO085_PortAsyncStatus_t BNO085_Port_SPITransferAsyncStatus(void);

/** Abort a pending async transfer during reset/recovery. / 复位恢复时终止异步传输。 */
void BNO085_Port_SPITransferAsyncAbort(void);

/** Return and clear neither the normalized nor raw last error. / 查询最近错误，不清除。 */
BNO085_PortError_t BNO085_Port_GetLastError(uint32_t *raw_error);

/** Reinitialize the SPI/DMA peripheral after a transport fault. / 总线故障后重建外设。 */
bool BNO085_Port_Recover(void);

/** Assert/deassert active-low H_CSN. / 拉低或释放 H_CSN。 */
void BNO085_Port_SetChipSelect(bool asserted);
/** Assert/deassert active-low WAKE/PS0. / 拉低或释放 WAKE/PS0。 */
void BNO085_Port_SetWake(bool asserted);
/** Assert/deassert active-low NRST. / 拉低或释放 NRST。 */
void BNO085_Port_SetReset(bool asserted);
/** Return true when active-low H_INTN is low. / H_INTN 为低时返回 true。 */
bool BNO085_Port_IsInterruptAsserted(void);
/** Monotonic millisecond tick; natural uint32 wrap is allowed. / 单调毫秒时基。 */
uint32_t BNO085_Port_GetTimeMs(void);
/** Blocking millisecond delay used only during setup/recovery. / 初始化用阻塞延时。 */
void BNO085_Port_DelayMs(uint32_t delay_ms);

#endif
