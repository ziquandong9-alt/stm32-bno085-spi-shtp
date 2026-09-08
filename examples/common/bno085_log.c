/* SPDX-License-Identifier: BSD-3-Clause */
#include "bno085_log.h"

#include <stdarg.h>
#include <stdio.h>

#define BNO085_LOG_BUFFER_SIZE 160U

static BNO085_LogSink_t log_sink;
static void *log_context;
static BNO085_LogLevel_t log_maximum_level = BNO085_LOG_INFO;

void BNO085_Log_Init(BNO085_LogSink_t sink, void *user_context,
                     BNO085_LogLevel_t maximum_level)
{
    log_sink = sink;
    log_context = user_context;
    log_maximum_level = maximum_level;
}

void BNO085_Log_Write(BNO085_LogLevel_t level, const char *format, ...)
{
    char buffer[BNO085_LOG_BUFFER_SIZE];
    int length;
    va_list arguments;

    if ((log_sink == NULL) || (format == NULL) ||
        (level > log_maximum_level)) return;
    va_start(arguments, format);
    length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);
    if (length < 0) return;
    if ((size_t)length >= sizeof(buffer)) length = (int)sizeof(buffer) - 1;
    log_sink(level, buffer, (size_t)length, log_context);
}
