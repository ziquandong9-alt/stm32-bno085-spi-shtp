/* SPDX-License-Identifier: BSD-3-Clause */
/* Minimal application pattern; provide board_init() and your MCU Port layer. */
#include "bno085.h"

#include <stddef.h>

static volatile bool attitude_updated;
static BNO085_Euler_t attitude;

static void on_bno085_data(uint32_t events, void *context)
{
    (void)context;
    if ((events & BNO085_EVENT_ROTATION_VECTOR) != 0U) {
        if (BNO085_GetEuler(&attitude) == BNO085_OK) attitude_updated = true;
    }
}

void app_main(void)
{
    BNO085_Callbacks_t callbacks = { on_bno085_data, NULL, NULL };
    BNO085_ReportConfig_t rv = { 10000U, 0U, 0U, 0U, 0U };

    /* Configure the board-specific port before this point. */
    BNO085_SetCallbacks(&callbacks);
    if (BNO085_Init() != BNO085_OK) return;
    if (BNO085_ConfigureReport(BNO085_REPORT_ROTATION_VECTOR, &rv) !=
        BNO085_OK) return;

    for (;;) {
        (void)BNO085_Process(NULL); /* Never waits for DMA completion. */
        if (attitude_updated) {
            attitude_updated = false;
            /* Consume attitude here; the getter never performs SPI I/O. */
        }
        /* Run other work or enter a board-specific idle policy here. */
    }
}
