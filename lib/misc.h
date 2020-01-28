/**
 * 非专门型号传感器的驱动和一些处理函数先放这里.
 */

#ifndef _MISC_H
#define _MISC_H

#include <stddef.h>

/** 测速码盘最大记录数 */
#define COUNTER_MAX_RECORD  4
/** 测速时间窗口，单位毫秒，只使用这个时间内的计数计算速度 */
#define COUNTER_SPEED_WINDOW    100
/** 理论最大速度 */
#define COUNTER_MAX_SPEED   1000


/**
 * 测速码盘.
 */
typedef struct
{
    uint32_t min_interval;                          /**< 两次计数间最小时间间隔，单位毫秒，用于消抖 */
    volatile uint32_t count;                        /**< 当前计数 */
    volatile uint32_t record[COUNTER_MAX_RECORD];   /**< 历史计数记录 */
    volatile uint32_t *prev;                        /**< 指向上一条记录 */
    volatile uint32_t *next;                        /**< 指向下一条记录 */
} Counter;

/**
 * 初始化计数.
 */
extern int counter_init(Counter *counter, uint32_t min_interval);

/**
 * 清空除计数意外的记录.
 * 
 * 通常在码盘改变转向时调用.
 */
extern int counter_clear(Counter *counter);

/**
 * 检测到一次计数，由计数中断调用.
 */
extern void counter_inc(Counter *counter);

/**
 * 读取当前计数.
 */
static __INLINE uint32_t counter_read(const volatile Counter *counter)
{
    return counter->count;
}

/**
 * 根据计数记录计算实时转速.
 * 
 * 计算测速时间窗口内的平均速度，单位0.1格/秒.
 */
extern uint32_t counter_get_speed(const volatile Counter *counter);

#endif  /* _MISC_H */
