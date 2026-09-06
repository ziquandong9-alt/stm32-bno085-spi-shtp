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
    return (HAL_SPI_TransmitReceive(port_config.spi, (uint8_t *)tx, rx,
                                    length, timeout_ms) == HAL_OK);
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
