/**
 * 用户输出日志的库函数.
 */

#ifndef _LOG_H
#define _LOG_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>


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
    LOG_FATAL,
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

/**
 * 设置日志缓冲区大小，决定一次能处理的日志长度.
 */
#ifndef LOG_BUFFER_LEN
#define LOG_BUFFER_LEN 256
#endif  /* LOG_BUFFER_LEN */

/**
 * 实际输出一条日志字符串到设备.
 */
extern uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len);
#define _log_output(buf, len)  CDC_Transmit_FS((uint8_t*)(buf), len)

/**
 * 输出一条日志，日志实际输出的设备需要配置.
 */
__STATIC_INLINE void _log_log(LogLevel level, const char *msg, ...)
{
    if (level >= LOG_LEVEL)
    {
        static char buf[LOG_BUFFER_LEN];
        int len = 0;
        int n;
        char flag;

        switch (level)
        {
        case LOG_VERBOSE:
            flag = 'V';
            break;
        case LOG_DEBUG:
            flag = 'D';
            break;
        case LOG_INFO:
            flag = 'I';
            break;
        case LOG_ERROR:
            flag = 'E';
            break;
        case LOG_FATAL:
            flag = 'F';
            break;
        default:
            flag = ' ';
            break;
        }

        n = snprintf(buf, LOG_BUFFER_LEN, "[%c] ", flag);
        if ((n >= 0) && (n < LOG_BUFFER_LEN))
        {
            len = n;
        }
        else
        {
            return;
        }

        va_list args;
        va_start(args, msg);
        n = vsnprintf(buf + len, LOG_BUFFER_LEN - len, msg, args);
        va_end(args);

        if (n >= 0)
        {
            len += n;
        }
        else
        {
            return;
        }

        if (len < LOG_BUFFER_LEN)
        {
            buf[len] = '\n';
            ++len;
        }

        _log_output(buf, len);
    }
}

/*
 * 一些简便日志接口
 */
#define log_log(level, msg, ...)    _log_log((level), "%s:%s:%d: " msg, __FILE__, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__)

#define log_verbose(...)    log_log(LOG_VERBOSE, __VA_ARGS__)
#define log_debug(...)      log_log(LOG_DEBUG, __VA_ARGS__)
#define log_info(...)       log_log(LOG_INFO, __VA_ARGS__)
#define log_warn(...)       log_log(LOG_WARN, __VA_ARGS__)
#define log_error(...)      log_log(LOG_ERROR, __VA_ARGS__)
#define log_fatal(...)      log_log(LOG_FATAL, __VA_ARGS__)

#endif  /* _LOG_H */
