/* SPDX-License-Identifier: BSD-3-Clause */
/* Private test seam; never installed or included by application code.
 * 私有测试接缝；不会安装，也不会被应用代码包含。 */
#ifndef BNO085_TEST_HOOKS_H
#define BNO085_TEST_HOOKS_H

#include <stdint.h>

#include "bno085.h"

BNO085_Status_t BNO085_Test_ParseSensorPayload(const uint8_t *payload,
                                                uint16_t length,
                                                uint32_t packet_timestamp_us,
                                                uint32_t *events);
void BNO085_Test_ResetState(void);
void BNO085_Test_SetTransportActive(void);

#endif
