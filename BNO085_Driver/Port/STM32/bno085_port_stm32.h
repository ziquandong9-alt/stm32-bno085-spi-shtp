/* SPDX-License-Identifier: BSD-3-Clause */
/**
 * @file bno085_port_stm32.h
 * @brief STM32 HAL binding for the portable BNO085 core.
 *        可移植 BNO085 核心的 STM32 HAL 绑定层。
 */
#ifndef BNO085_PORT_STM32_H
#define BNO085_PORT_STM32_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** STM32 peripheral and GPIO mapping copied by the adapter. / STM32 硬件映射。 */
typedef struct {
    SPI_HandleTypeDef *spi;       /**< SPI Mode 3 master handle. / Mode 3 主机句柄。 */
    GPIO_TypeDef *cs_port;        /**< H_CSN GPIO port. / 片选端口。 */
    uint16_t cs_pin;
    GPIO_TypeDef *wake_port;
    uint16_t wake_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *interrupt_port;
    uint16_t interrupt_pin;
} BNO085_STM32_PortConfig_t;

/** Validate and copy the pin mapping; does not reset BNO085. / 校验并复制引脚配置。 */
bool BNO085_STM32_Port_Init(const BNO085_STM32_PortConfig_t *config);

#endif
