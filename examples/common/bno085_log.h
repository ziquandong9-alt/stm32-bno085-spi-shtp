/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef BNO085_APP_LOG_H
#define BNO085_APP_LOG_H

#include <stddef.h>

typedef enum {
    BNO085_LOG_ERROR = 0,
    BNO085_LOG_WARNING,
    BNO085_LOG_INFO,
    BNO085_LOG_DEBUG
} BNO085_LogLevel_t;

/** Platform sink: enqueue to UART DMA, RTT, USB CDC, or discard it.
 *  平台输出端：可接 UART DMA、RTT、USB CDC，或直接丢弃。 */
typedef void (*BNO085_LogSink_t)(BNO085_LogLevel_t level,
                                 const char *message,
                                 size_t length,
                                 void *user_context);

void BNO085_Log_Init(BNO085_LogSink_t sink, void *user_context,
                     BNO085_LogLevel_t maximum_level);
void BNO085_Log_Write(BNO085_LogLevel_t level, const char *format, ...);

#endif
