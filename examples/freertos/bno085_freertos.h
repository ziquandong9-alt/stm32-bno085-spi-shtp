/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef BNO085_FREERTOS_EXAMPLE_H
#define BNO085_FREERTOS_EXAMPLE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "bno085.h"

/** Create before starting the scheduler; the Port layer must already exist. */
BaseType_t BNO085_FreeRTOS_Start(UBaseType_t priority,
                                 uint16_t stack_depth_words);
/** Call from the BNO085 H_INTN EXTI callback. / 在 H_INTN 外部中断中调用。 */
void BNO085_FreeRTOS_NotifyFromISR(void);
/** Queue contains complete, coherent Euler snapshots. / 队列保存完整欧拉角快照。 */
QueueHandle_t BNO085_FreeRTOS_GetAttitudeQueue(void);

#endif
