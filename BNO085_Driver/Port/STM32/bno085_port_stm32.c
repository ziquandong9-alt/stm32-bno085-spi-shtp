/* SPDX-License-Identifier: BSD-3-Clause */
/**
 * @file bno085_port_stm32.c
 * @brief STM32F4 HAL implementation of bno085_port.h.
 *        bno085_port.h 的 STM32F4 HAL 实现。
 *
 * This is the only driver source that calls STM32 HAL.  When changing MCU,
 * replace this file rather than modifying the SHTP/SH-2 core.
 * 这是驱动中唯一调用 STM32 HAL 的源文件；更换 MCU 时替换本文件即可。
 */
#include "bno085_port_stm32.h"
#include "bno085_port.h"

/* The portable core is a single instance, so one copied mapping is enough.
 * 核心为单实例设计，因此保存一份引脚和 SPI 句柄即可。 */
static BNO085_STM32_PortConfig_t port_config;
static bool port_ready;
/* Written in HAL IRQ callbacks and read in thread/main context. / ISR 写、主循环读。 */
static volatile BNO085_PortAsyncStatus_t async_status = BNO085_PORT_ASYNC_IDLE;
static volatile BNO085_PortError_t last_error = BNO085_PORT_ERROR_NONE;
static volatile uint32_t last_raw_error;

/** Translate HAL state without leaking HAL types into the portable core. */
static void capture_hal_error(HAL_StatusTypeDef status, bool dma_operation)
{
    last_raw_error = HAL_SPI_GetError(port_config.spi);
    if (status == HAL_BUSY) {
        last_error = BNO085_PORT_ERROR_BUSY;
    } else if (status == HAL_TIMEOUT) {
        last_error = BNO085_PORT_ERROR_TIMEOUT;
    } else {
        last_error = dma_operation ? BNO085_PORT_ERROR_DMA :
                                     BNO085_PORT_ERROR_SPI;
    }
}

bool BNO085_STM32_Port_Init(const BNO085_STM32_PortConfig_t *config)
{
    if ((config == NULL) || (config->spi == NULL) ||
        (config->cs_port == NULL) || (config->wake_port == NULL) ||
        (config->reset_port == NULL) ||
        (config->interrupt_port == NULL)) {
        return false;
    }

    /* Copy instead of storing the caller's pointer, allowing a local config.
     * 复制结构体，调用者可以安全地使用局部变量传入配置。 */
    port_config = *config;
    port_ready = true;
    async_status = BNO085_PORT_ASYNC_IDLE;
    last_error = BNO085_PORT_ERROR_NONE;
    last_raw_error = 0U;
    BNO085_Port_SetChipSelect(false);
    BNO085_Port_SetWake(false);
    BNO085_Port_SetReset(false);
    return true;
}

bool BNO085_Port_IsReady(void)
{
    return port_ready;
}

bool BNO085_Port_SPITransfer(const uint8_t *tx, uint8_t *rx,
                            uint16_t length, uint32_t timeout_ms)
{
    if ((!port_ready) || (tx == NULL) || (rx == NULL) || (length == 0U)) {
        return false;
    }
    /* BNO085 SPI is full duplex.  HAL requires non-const TxData, hence the
     * cast; this function never modifies the transmit buffer itself.
     * BNO085 SPI 为全双工；HAL 参数不是 const，所以这里仅做类型转换。 */
    {
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
            port_config.spi, (uint8_t *)tx, rx, length, timeout_ms);
        if (status != HAL_OK) {
            capture_hal_error(status, false);
            return false;
        }
    }
    last_error = BNO085_PORT_ERROR_NONE;
    last_raw_error = 0U;
    return true;
}

bool BNO085_Port_SPITransferAsync(const uint8_t *tx, uint8_t *rx,
                                 uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    if ((!port_ready) || (tx == NULL) || (rx == NULL) || (length == 0U) ||
        (async_status == BNO085_PORT_ASYNC_BUSY)) {
        return false;
    }

    /* Publish BUSY before enabling DMA interrupts so even a very short
     * transfer cannot complete before the state is visible.
     * 先发布 BUSY 再开 DMA，避免极短事务完成中断与状态写入竞争。 */
    async_status = BNO085_PORT_ASYNC_BUSY;
    /* HAL also declares DMA TxData non-const; DMA only reads this buffer.
     * HAL 的 DMA TxData 同样未标 const，但 DMA 只会读取该缓冲。 */
    hal_status = HAL_SPI_TransmitReceive_DMA(port_config.spi, (uint8_t *)tx,
                                             rx, length);
    if (hal_status != HAL_OK) {
        capture_hal_error(hal_status, true);
        async_status = BNO085_PORT_ASYNC_ERROR;
        return false;
    }
    return true;
}

BNO085_PortAsyncStatus_t BNO085_Port_SPITransferAsyncStatus(void)
{
    return async_status;
}

void BNO085_Port_SPITransferAsyncAbort(void)
{
    if (port_ready && (async_status == BNO085_PORT_ASYNC_BUSY)) {
        (void)HAL_SPI_DMAStop(port_config.spi);
    }
    async_status = BNO085_PORT_ASYNC_IDLE;
}

BNO085_PortError_t BNO085_Port_GetLastError(uint32_t *raw_error)
{
    if (raw_error != NULL) {
        *raw_error = last_raw_error;
    }
    return last_error;
}

bool BNO085_Port_Recover(void)
{
    HAL_StatusTypeDef status;

    if (!port_ready) {
        return false;
    }
    BNO085_Port_SetChipSelect(false);
    BNO085_Port_SetWake(false);
    (void)HAL_SPI_DMAStop(port_config.spi);
    status = HAL_SPI_Abort(port_config.spi);
    if ((status != HAL_OK) && (status != HAL_ERROR)) {
        capture_hal_error(status, false);
        return false;
    }
    if (HAL_SPI_DeInit(port_config.spi) != HAL_OK) {
        capture_hal_error(HAL_ERROR, false);
        return false;
    }
    if (HAL_SPI_Init(port_config.spi) != HAL_OK) {
        capture_hal_error(HAL_ERROR, false);
        return false;
    }
    async_status = BNO085_PORT_ASYNC_IDLE;
    last_error = BNO085_PORT_ERROR_NONE;
    last_raw_error = 0U;
    return true;
}

/** HAL full-duplex completion hook. / HAL 全双工 DMA 完成回调。 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (port_ready && (hspi == port_config.spi)) {
        async_status = BNO085_PORT_ASYNC_COMPLETE;
    }
}

/** HAL SPI/DMA error hook. / HAL SPI 或 DMA 错误回调。 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (port_ready && (hspi == port_config.spi)) {
        capture_hal_error(HAL_ERROR, true);
        async_status = BNO085_PORT_ASYNC_ERROR;
    }
}

void BNO085_Port_SetChipSelect(bool asserted)
{
    if (port_ready) {
        HAL_GPIO_WritePin(port_config.cs_port, port_config.cs_pin,
                          asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

void BNO085_Port_SetWake(bool asserted)
{
    if (port_ready) {
        HAL_GPIO_WritePin(port_config.wake_port, port_config.wake_pin,
                          asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

void BNO085_Port_SetReset(bool asserted)
{
    if (port_ready) {
        HAL_GPIO_WritePin(port_config.reset_port, port_config.reset_pin,
                          asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

bool BNO085_Port_IsInterruptAsserted(void)
{
    if (!port_ready) {
        return false;
    }
    return (HAL_GPIO_ReadPin(port_config.interrupt_port,
                            port_config.interrupt_pin) == GPIO_PIN_RESET);
}

uint32_t BNO085_Port_GetTimeMs(void)
{
    return HAL_GetTick();
}

void BNO085_Port_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}
