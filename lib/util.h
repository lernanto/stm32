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

#endif  /* _UTIL_H */
