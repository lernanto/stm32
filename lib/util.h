/**
 * 公共库函数.
 */

#ifndef _UTIL_H
#define _UTIL_H

#include "stm32f1xx_hal.h"


/**
 * 返回数组的元素个数.
 */
#define ARRAYSIZE(x)    (sizeof(x) / sizeof((x)[0]))

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

/**
 * 延时指定微秒数.
 */
__STATIC_INLINE uint32_t delayus(uint32_t us)
{
    uint32_t startms;
    int32_t startval;
    uint32_t endms;
    int32_t endval;
    uint32_t timeus;

    if (0 == us)
    {
        return 0;
    }

    startms = HAL_GetTick();
    startval = SysTick->VAL;

    do
    {
        endms = HAL_GetTick();
        endval = SysTick->VAL;
        timeus = (endms - startms) * 1000 + (startval - endval) / 72;
    } while (timeus < us);

    return timeus;
}

#endif  /* _UTIL_H */
