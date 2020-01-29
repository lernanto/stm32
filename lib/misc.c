/**
 *
 */

#include <string.h>

#include "stm32f1xx.h"

#include "misc.h"


#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

/**
 * 计算3阶行列式.
 */
#define det3(a11, a12, a13, a21, a22, a23, a31, a32, a33) \
    ((a11) * ((a22) * (a33) - (a23) * (a32)) \
        - (a12) * ((a21) * (a33) - (a23) * (a31)) \
        + (a13) * ((a21) * (a32) - (a22) * (a31)))


int counter_init(Counter *counter, uint32_t min_interval)
{
    counter->min_interval = min_interval;
    counter->count = 0;
    memset((void *)counter->record, 0, sizeof(counter->record));
    counter->prev = NULL;
    counter->next = counter->record;

    return 1;
}

int counter_clear(Counter *counter)
{
    memset((void *)counter->record, 0, sizeof(counter->record));
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
 * 计算码盘实时计数、转速及加速度.
 * 
 * 使用二次曲线的最小二乘法拟合记录样本.
 */
int counter_get_state(
    const volatile Counter *counter,
    uint32_t *time,
    uint32_t *count,
    int32_t *speed,
    int32_t *acc
)
{
    uint32_t xbase = 0;
    uint32_t n = 0;
    int64_t sx = 0;
    int64_t sx2 = 0;
    int64_t sx3 = 0;
    int64_t sx4 = 0;
    int64_t sy = 0;
    int64_t sxy = 0;
    int64_t sx2y = 0;
    const volatile uint32_t *p = NULL;

    *time = HAL_GetTick();
    *count = counter->count;
    xbase = MAX(*counter->next, *time - COUNTER_STATE_WINDOW);

    if (NULL == counter->prev)
    {
        *speed = 0;
        *acc = 0;
        return 1;
    }

    /* 逆序遍历计数记录查找测速时间窗口内的记录 */
    p = counter->prev;
    for (
        uint32_t y = COUNTER_MAX_RECORD;
        (y > 0) && (*p != 0) && (*p + COUNTER_STATE_WINDOW > *time);
        --y
    )
    {
        /* 为避免溢出，时间和计数统一减去一个基数 */
        uint32_t x = *p - xbase;
        int64_t x2 = x * x;

        ++n;
        sx += x;
        sx2 += x2;
        sx3 += x2 * x;
        sx4 += x2 * x2;
        sy += y;
        sxy += x * y;
        sx2y += x2 * y;

        if (--p < counter->record)
        {
            p = counter->record + COUNTER_MAX_RECORD - 1;
        }
    }

    if (n < 2)
    {
        /* 时间窗口内记录数不足以计算速度，认为速度和加速度为0 */
        *speed = 0;
        *acc = 0;
    }
    else if (2 == n)
    {
        /* 只有2个样本点，退化为线性插值 */
        uint32_t x1 = ((counter->prev > counter->record)
            ? *(counter->prev - 1)
            : counter->record[COUNTER_MAX_RECORD - 1]);
        uint32_t x2 = *counter->prev;

        *speed = (int32_t)(1000 / (x2 - x1));
        *acc = 0;
    }
    else
    {
        /* 使用二次曲线拟合样本点 */
        /* 使用最小二乘法计算二次曲线参数 */
        int64_t d = det3(sx4, sx3, sx2, sx3, sx2, sx, sx2, sx, n);

        if (d != 0)
        {
            int64_t w2 = det3(sx2y, sxy, sy, sx3, sx2, sx, sx2, sx, n);
            int64_t w1 = det3(sx4, sx3, sx2, sx2y, sxy, sy, sx2, sx, n);
            int64_t w0 = det3(sx4, sx3, sx2, sx3, sx2, sx, sx2y, sxy, sy);
            UNUSED(w0);

            /* 根据算得得二次曲线计算当前计数、速度和加速度 */
            *speed = (int32_t)((w2 * (*time - xbase) + w1) * 1000 / d);
            *acc = (int32_t)(w2 * 1000000 / d);
        }
        else
        {
            /* 系数矩阵不满秩，退化为线性插值 */
            uint32_t x1 = ((counter->prev > counter->record)
                ? *(counter->prev - 1)
                : counter->record[COUNTER_MAX_RECORD - 1]);
            uint32_t x2 = *counter->prev;

            *speed = (int32_t)(1000 / (x2 - x1));
            *acc = 0;
        }
    }

    return 1;
}
