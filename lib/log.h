/**
 * 用户输出日志的库函数.
 */

#ifndef _LOG_H
#define _LOG_H

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#ifdef __arm__
#include "stm32f1xx_hal.h"

#define _log_get_timestamp() (HAL_GetTick())
#else
#include <time.h>

#define _log_get_timestamp() ((unsigned)time(NULL))
#endif  /* __arm__ */


/**
 * 定义日志级别
 */
typedef enum
{
    LOG_VERBOSE = 0,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
} LogLevel;

/**
 * 设置全局日志级别.
 */
#ifndef LOG_LEVEL
#ifdef NDEBUG
#define LOG_LEVEL   LOG_INFO
#else   /* NDEBUG */
#define LOG_LEVEL   LOG_DEBUG
#endif  /* NDEBUG */
#endif  /* LOG_LEVEL */

#ifndef LOG_SOURCE
#if LOG_LEVEL <= LOG_DEBUG
#define LOG_SOURCE 1
#else
#define LOG_SOURCE 0
#endif  /* LOG_LEVEL <= LOG_DEBUG */
#endif  /* LOG_SOURCE */

/**
 * 设置日志缓冲区大小，决定一次能处理的日志长度.
 */
#ifndef LOG_BUFFER_LEN
#define LOG_BUFFER_LEN 256
#endif  /* LOG_BUFFER_LEN */

/**
 * 实际输出一条日志字符串到设备，实际输出日志时应重新实现此函数.
 */
extern size_t _log_write(const void *buf, size_t len);

/**
 * 断言失败退出，使用断言应重新实现此函数.
 */
extern void _log_abort(void);

/**
 * 输出一条日志，日志实际输出的设备需要配置.
 */
static inline int _log_log(
    LogLevel level,
    const char *flag,
    unsigned timestamp,
    int src,
    const char *file,
    const char *func,
    int line,
    const char *msg, ...
)
{
    static char buf[LOG_BUFFER_LEN];
    size_t len = 0;
    int n = 0;

    /* 输出预定义的头部和时间 */
    n = snprintf(buf, sizeof(buf), "[%s] %u ", flag, timestamp);
    if (n < 0)
    {
        return n;
    }
    len = (size_t)n;
    if (len >= sizeof(buf))
    {
        return (int)_log_write(buf, len);
    }

    if (src)
    {
        /* 输出源代码信息便于调试 */
        n = snprintf(buf + len, sizeof(buf) - len, "%s:%s:%d: ", file, func, line);
        if (n < 0)
        {
            return n;
        }
        len += n;
        if (len >= sizeof(buf))
        {
            return (int)_log_write(buf, len);
        }
    }

    va_list args;
    va_start(args, msg);
    n = vsnprintf(buf + len, sizeof(buf) - len, msg, args);
    va_end(args);

    if (n < 0)
    {
        return n;
    }
    len += n;
    if (len < sizeof(buf))
    {
        buf[len] = '\n';
        ++len;
    }

    return (int)_log_write(buf, len);
}

/**
 * 输出日志的基本接口.
 */
#define log_log(level, flag, src, ...) \
do { if ((level) >= LOG_LEVEL) \
    _log_log((level), (flag), _log_get_timestamp(), (src), \
    __FILE__, __func__, __LINE__, __VA_ARGS__); \
} while (0)

/*
 * 一些简便日志接口
 */
#define log_verbose(...)    log_log(LOG_VERBOSE, "V", (LOG_SOURCE), __VA_ARGS__)
#define log_debug(...)      log_log(LOG_DEBUG, "D", (LOG_SOURCE),  __VA_ARGS__)
#define log_info(...)       log_log(LOG_INFO, "I", (LOG_SOURCE),  __VA_ARGS__)
#define log_warn(...)       log_log(LOG_WARN, "W", (LOG_SOURCE),  __VA_ARGS__)
#define log_error(...)      log_log(LOG_ERROR, "E", (LOG_SOURCE),  __VA_ARGS__)

#ifdef NDEBUG
#define log_assert(con)
#else
/**
 * 断言，当条件不满足时输出错误并退出.
 */
#define log_assert(con) \
do { \
    if (!(con)) \
    { \
        log_log(LOG_ERROR, "E", 1, "assertion failed: " #con); \
        _log_abort(); \
    } \
} while (0)
#endif  /* NDEBUG */

#endif  /* _LOG_H */
