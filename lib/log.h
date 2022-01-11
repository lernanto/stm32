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
 * 日志缓冲区.
 */
extern char g_log_buffer[LOG_BUFFER_LEN];
extern char *g_log_buffer_ptr;

extern int log_header(
    LogLevel level,
    unsigned timestamp,
    int src,
    const char *file,
    const char *func,
    int line
);

/**
 * 实际输出一条日志字符串到设备，实际输出日志时应重新实现此函数.
 */
extern size_t _log_write(const void *buf, size_t len);

/**
 * 断言失败退出，使用断言应重新实现此函数.
 */
extern void _log_abort(void);


#define log_begin(level) do { \
    if ((level) >= LOG_LEVEL) \
    { \
        log_header((level), _log_get_timestamp(), LOG_SOURCE, __FILE__, __func__, __LINE__); \
    } \
} while (0)

#define log_write(level, ...) do { \
    if ((level) >= LOG_LEVEL) \
    { \
        int n = snprintf( \
            g_log_buffer_ptr, \
            g_log_buffer + sizeof(g_log_buffer) - g_log_buffer_ptr, \
            __VA_ARGS__ \
        ); \
        if (n > 0) \
        { \
            g_log_buffer_ptr += n; \
        } \
    } \
} while (0)

#define log_end(level) do { \
    if ((level) >= LOG_LEVEL) \
    { \
        if (g_log_buffer_ptr < g_log_buffer + sizeof(g_log_buffer)) \
        { \
            *g_log_buffer_ptr++ = '\n'; \
        } \
        _log_write(g_log_buffer, g_log_buffer_ptr - g_log_buffer); \
    } \
} while (0)

/**
 * 输出日志的基本接口.
 */
#define log_log(level, ...) do { \
    log_begin((level)); \
    log_write((level), __VA_ARGS__); \
    log_end((level)); \
} while (0)

/*
 * 一些简便日志接口
 */
#define log_verbose(...)    log_log(LOG_VERBOSE, __VA_ARGS__)
#define log_debug(...)      log_log(LOG_DEBUG, __VA_ARGS__)
#define log_info(...)       log_log(LOG_INFO, __VA_ARGS__)
#define log_warn(...)       log_log(LOG_WARN, __VA_ARGS__)
#define log_error(...)      log_log(LOG_ERROR, __VA_ARGS__)

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
        int n; \
        log_header(LOG_ERROR, _log_get_timestamp(), 1, __FILE__, __func__, __LINE__); \
        if ((n = snprintf( \
            g_log_buffer_ptr, \
            g_log_buffer + sizeof(g_log_buffer) - g_log_buffer_ptr, \
            #con \
        )) > 0) \
        { \
            g_log_buffer_ptr += n; \
        } \
        if (g_log_buffer_ptr < g_log_buffer + sizeof(g_log_buffer)) \
        { \
            *g_log_buffer_ptr++ = '\n'; \
        } \
        _log_write(g_log_buffer, g_log_buffer_ptr - g_log_buffer); \
        _log_abort(); \
    } \
} while (0)
#endif  /* NDEBUG */

#endif  /* _LOG_H */
