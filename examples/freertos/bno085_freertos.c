/* SPDX-License-Identifier: BSD-3-Clause */
#include "bno085_freertos.h"

static TaskHandle_t sensor_task;
static QueueHandle_t attitude_queue;

static void on_data(uint32_t events, void *context)
{
    BNO085_Euler_t value;
    (void)context;
    if (((events & BNO085_EVENT_ROTATION_VECTOR) != 0U) &&
        (BNO085_GetEuler(&value) == BNO085_OK)) {
        /* Length one deliberately keeps only the newest control sample. */
        (void)xQueueOverwrite(attitude_queue, &value);
    }
}

static void sensor_task_entry(void *argument)
{
    BNO085_Callbacks_t callbacks = { on_data, NULL, NULL };
    BNO085_ReportConfig_t report = { 10000U, 0U, 0U, 0U, 0U };
    (void)argument;

    BNO085_SetCallbacks(&callbacks);
    if ((BNO085_Init() != BNO085_OK) ||
        (BNO085_ConfigureReport(BNO085_REPORT_ROTATION_VECTOR, &report) !=
         BNO085_OK)) {
        vTaskSuspend(NULL);
    }

    for (;;) {
        /* EXTI wakes immediately; timeout also services a DMA completion when
         * the MCU port does not notify separately. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2U));
        while (BNO085_Process(NULL) == BNO085_OK) { }
    }
}

BaseType_t BNO085_FreeRTOS_Start(UBaseType_t priority,
                                 uint16_t stack_depth_words)
{
    attitude_queue = xQueueCreate(1U, sizeof(BNO085_Euler_t));
    if (attitude_queue == NULL) return pdFAIL;
    return xTaskCreate(sensor_task_entry, "bno085", stack_depth_words, NULL,
                       priority, &sensor_task);
}

void BNO085_FreeRTOS_NotifyFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (sensor_task != NULL) {
        vTaskNotifyGiveFromISR(sensor_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

QueueHandle_t BNO085_FreeRTOS_GetAttitudeQueue(void)
{
    return attitude_queue;
}
