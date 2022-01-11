/**
 *
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "log.h"


#ifndef __GNUC__
#define __attribute__(attr)
#endif  /* __GNUC__ */


char g_log_buffer[LOG_BUFFER_LEN];
char *g_log_buffer_ptr;

int log_header(
    LogLevel level,
    unsigned timestamp,
    int src,
    const char *file,
    const char *func,
    int line
)
{
    char *flag;
    int n;

    switch (level)
    {
    case LOG_VERBOSE:
        flag = "V";
        break;
    case LOG_DEBUG:
        flag = "D";
        break;
    case LOG_INFO:
        flag = "I";
        break;
    case LOG_WARN:
        flag = "W";
        break;
    case LOG_ERROR:
        flag = "E";
        break;
    default:
        flag = "";
        break;
    }

    /* 输出预定义的头部和时间 */
    n = snprintf(g_log_buffer, sizeof(g_log_buffer), "[%s] %u ", flag, timestamp);
    if (n < 0)
    {
        return n;
    }
    g_log_buffer_ptr = g_log_buffer + n;
    if (g_log_buffer_ptr >= g_log_buffer + sizeof(g_log_buffer))
    {
        return (int)sizeof(g_log_buffer);
    }

    if (src)
    {
        /* 输出源代码信息便于调试 */
        n = snprintf(
            g_log_buffer_ptr,
            g_log_buffer + sizeof(g_log_buffer) - g_log_buffer_ptr,
            "%s:%s:%d: ",
            file,
            func,
            line
        );
        if (n < 0)
        {
            return n;
        }
        g_log_buffer_ptr += n;
        if (g_log_buffer_ptr >= g_log_buffer + sizeof(g_log_buffer))
        {
            return (int)sizeof(g_log_buffer);
        }
    }

    return g_log_buffer_ptr - g_log_buffer;
}

__attribute__((weak)) size_t _log_write(const void *buf, size_t len)
{
    return (size_t)fwrite(buf, 1, len, stderr);
}

__attribute__((weak, noreturn)) void _log_abort(void)
{
    abort();
}
