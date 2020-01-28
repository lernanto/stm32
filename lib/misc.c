/**
 *
 */

#include <string.h>

#include "stm32f1xx.h"

#include "misc.h"


int counter_init(Counter *counter, uint32_t min_interval)
{
    counter->min_interval = min_interval;
    counter->count = 0;
    memset(counter->record, 0, sizeof(counter->record));
    counter->prev = NULL;
    counter->next = counter->record;

    return 1;
}

int counter_clear(Counter *counter)
{
    memset(counter->record, 0, sizeof(counter->record));
    counter->prev = NULL;
    counter->next = counter->record;

    return 1;
}

void counter_inc(Counter *counter)
{
    /* 记录计数时间 */
    uint32_t now = HAL_GetTick();

    /* 软件消抖，必须大于理论最小间隔时间才计数 */
    if ((NULL == counter->prev) || (now > *counter->prev + counter->min_interval))
    {
        ++counter->count;
        *counter->next = now;
        counter->prev = counter->next;
        if (++counter->next >= counter->record + COUNTER_MAX_RECORD)
        {
            counter->next = counter->record;
        }
    }
}

/**
 * 计算码盘实时转速.
 * 
 * 计算测速时间窗口内的平均速度
 * TODO: 使用最小二乘法或其他插值方法拟合记录
 */
uint32_t counter_get_speed(const volatile Counter *counter)
{
    uint32_t now = HAL_GetTick();
    uint32_t count = 0;
    uint32_t min_time = UINT32_MAX;
    uint32_t max_time = 0;

    /* 遍历计数记录查找测速时间窗口内的最早和最晚记录 */
    for (const uint32_t *p = counter->record; p < counter->record + COUNTER_MAX_RECORD; ++p)
    {
        if ((0 != *p) && (*p + COUNTER_SPEED_WINDOW > now))
        {
            ++count;
            if (*p < min_time)
            {
                min_time = *p;
            }
            if (*p > max_time)
            {
                max_time = *p;
            }
        }
    }

    if (count < 2)
    {
        /* 时间窗口内记录数不足以计算速度，认为速度为0 */
        return 0;
    }
    else if (min_time >= max_time)
    {
        /* 计数更新太快，返回最快速度 */
        return COUNTER_MAX_SPEED;
    }
    else
    {
        /* 平均速度为窗口内计数除以时间差 */
        return (count - 1) * 10000 / (max_time - min_time);
    }
}
