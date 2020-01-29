/**
 * 非专门型号传感器的驱动和一些处理函数先放这里.
 */

#ifndef _MISC_H
#define _MISC_H

#include <stddef.h>

/** 测速码盘最大记录数 */
#define COUNTER_MAX_RECORD  5
/** 测速时间窗口，单位毫秒，只使用这个时间内的计数计算速度 */
#define COUNTER_STATE_WINDOW    100


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
 * 根据计数记录计算码盘实时计数、转速及加速度.
 * 
 * 使用二次曲线拟合测速时间窗口内的记录，再根据拟合的参数计算速度和加速度。
 * 返回速度单位为格/秒，加速度单位为0.001格/秒。
 */
extern int counter_get_state(
    const volatile Counter *counter,
    uint32_t *time,
    uint32_t *count,
    int32_t *speed,
    int32_t *acc
);

#endif  /* _MISC_H */
