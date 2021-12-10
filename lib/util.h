/**
 * @brief 公共库函数.
 */

#ifndef _UTIL_H
#define _UTIL_H

#include "stm32f1xx_hal.h"


/**
 * @brief 返回数组的元素个数.
 */
#define ARRAYSIZE(x)    (sizeof(x) / sizeof((x)[0]))

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

/**
 * 延时指定微秒数.
 */
__STATIC_INLINE void delayus(uint32_t us)
{
    uint32_t startms;
    uint32_t startval;
    uint32_t endms;
    uint32_t endval;
    uint32_t clk;

    if (0 == us)
    {
        return;
    }

    startms = HAL_GetTick();
    startval = SysTick->VAL;
    endms = startms + us / 1000;
    clk = us % 1000 * (HAL_RCC_GetHCLKFreq() / 1000000);
    if (startval < clk)
    {
        ++endms;
        startval += SysTick->LOAD + 1;
    }
    endval = startval - clk;

    while (HAL_GetTick() < endms);
    while (SysTick->VAL > endval);
}

/**
 * @brief 返回循环队列实际存储的元素数
 */
#define circular_size(begin, end, cap) \
    (((end) >= (begin)) ? ((end) - (begin)) : ((end) + (cap) - (begin)))

/**
 * @brief 递增循环队列的指针
 *
 * 如果指针到了缓冲区结尾，重新循环到开头
 */
#define circular_inc(buf, cap, p) \
    ((p) = (((p) < (buf) + (cap) - 1) ? ((p) + 1) : (buf)))

/**
 * @brief 添加元素到队列结尾
 *
 * 数据必须已经写入队列结尾，实际只是递增结尾指针
 * 如果结尾指针循环到队列开头，删除开头一个元素已留出空间
 */
#define circular_push(buf, cap, begin, end) do { \
    circular_inc((buf), (cap), (end)); \
    if ((end) == (begin)) circular_inc((buf), (cap), (begin)); \
} while (0)

/**
 * @brief 从循环队列开头删除一个元素
 */
#define circular_pop(buf, cap, begin, end) do { \
    if ((begin) != (end)) circular_inc((buf), (cap), (begin)); \
} while (0)

#endif  /* _UTIL_H */
