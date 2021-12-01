/**
 * @brief HC-SR04 超声测距传感器驱动.
 *
 * @see https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 */

#include "stm32f1xx_hal.h"

#include "util.h"
#include "hcsr04.h"


/**
 * 测量障碍物的距离，单位毫米. 
 */
__STATIC_INLINE uint32_t _get_dist(
    uint32_t begin_ms,
    uint32_t begin_tick,
    uint32_t end_ms,
    uint32_t end_tick
)
{
    uint32_t timeus = 0;
    uint32_t dist = 0;

    if ((end_ms < begin_ms)
        || ((end_ms == begin_ms) && (end_tick >= begin_tick)))
    {
        return HCSR04_INVALID_DISTANCE;
    }

    /* 根据记录计算 ECHO 高电平持续时间 */
    timeus = (end_ms - begin_ms) * 1000
        + ((int32_t)begin_tick - (int32_t)end_tick) / (HAL_RCC_GetHCLKFreq() / 1000000);

    /* 根据超声波在空气中的传播速度计算障碍物距离，注意测量的时间包含往返 */
    dist = timeus * HCSR04_ULTRASOUND_SPEED / 2 / 1000;
    return ((dist >= HCSR04_MIN_DISTANCE) && (dist <= HCSR04_MAX_DISTANCE)) ?
        dist : HCSR04_INVALID_DISTANCE;
}

__STATIC_INLINE uint32_t _read(const Hcsr04Control *hcsr04)
{
    /* 未接收到 ECHO 信号或信号未结束 */
    if ((hcsr04->state != HCSR04_ECHO_END)
        /* 距离上次 ECHO 信号结束已超过指定时间 */
        || (HAL_GetTick() > hcsr04->echo_end_ms + hcsr04->expire))
    {
        return HCSR04_INVALID_DISTANCE;
    }

    /* 根据记录的 ECHO 开始和结束时间计算持续时间，前者是中断设置的 */
    return _get_dist(
        hcsr04->echo_begin_ms,
        hcsr04->echo_begin_tick,
        hcsr04->echo_end_ms,
        hcsr04->echo_end_tick
    );
}

int hcsr04_init(
    Hcsr04Control *hcsr04,
    GPIO_TypeDef *trig_port,
    uint16_t trig_pin,
    GPIO_TypeDef *echo_port,
    uint16_t echo_pin,
    uint32_t expire
)
{
    hcsr04->trig_port = trig_port;
    hcsr04->trig_pin = trig_pin;
    hcsr04->echo_port = echo_port;
    hcsr04->echo_pin = echo_pin;
    hcsr04->expire = expire;
    hcsr04->state = HCSR04_RESET;
    hcsr04->echo_begin_ms = 0;
    hcsr04->echo_begin_tick = 0;
    hcsr04->echo_end_ms = 0;
    hcsr04->echo_end_tick = 0;

    if (hcsr04->expire < HCSR04_TIMEOUT + HCSR04_TIMEOUT)
    {
        hcsr04->expire = HCSR04_TIMEOUT + HCSR04_TIMEOUT;
    }

    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    return 1;
}

/**
 * 拉升 TRIG 引脚10微秒以上，发射超声波
 */
int hcsr04_trigger(Hcsr04Control *hcsr04)
{
    HAL_GPIO_WritePin(hcsr04->trig_port, hcsr04->trig_pin, GPIO_PIN_SET);
    delayus(20);
    HAL_GPIO_WritePin(hcsr04->trig_port, hcsr04->trig_pin, GPIO_PIN_RESET);

    return 1;
}

uint32_t hcsr04_get_dist_sync(Hcsr04Control *hcsr04)
{
    uint32_t begin_ms = 0;
    uint32_t begin_tick = 0;
    uint32_t end_ms = 0;
    uint32_t end_tick = 0;
    GPIO_PinState echo = GPIO_PIN_RESET;

    /* 发射超声波 */
    hcsr04_trigger(hcsr04);

    /* 等待 ECHO 引脚返回高电平 */
    begin_ms = HAL_GetTick();
    do
    {
        echo = HAL_GPIO_ReadPin(hcsr04->echo_port, hcsr04->echo_pin);
        begin_tick = SysTick->VAL;
        end_ms = HAL_GetTick();
    } while ((GPIO_PIN_RESET == echo) && (end_ms <= begin_ms + HCSR04_TIMEOUT));

    if (GPIO_PIN_RESET == echo)
    {
        /* 超时，检测不到障碍物 */
        return HCSR04_INVALID_DISTANCE;
    }

    /* 检测到 ECHO 高电平，等待高电平结束，计算持续时间 */
    begin_ms = end_ms;
    do
    {
        echo = HAL_GPIO_ReadPin(hcsr04->echo_port, hcsr04->echo_pin);
        end_tick = SysTick->VAL;
        end_ms = HAL_GetTick();
    } while ((GPIO_PIN_SET == echo) && (end_ms <= begin_ms + HCSR04_TIMEOUT));

    if (GPIO_PIN_SET == echo)
    {
        /* 超时，障碍物距离超量程 */
        return HCSR04_INVALID_DISTANCE;
    }

    return _get_dist(begin_ms, begin_tick, end_ms, end_tick);
}

/**
 * ECHO 引脚上升沿触发中断.
 *
 * 记录开始时间和 SysTick 寄存器当前值，用于后面计算持续持续时间.
 */
void hcsr04_echo_begin(Hcsr04Control *hcsr04)
{
    hcsr04->state = HCSR04_ECHO_BEGIN;
    hcsr04->echo_begin_tick = SysTick->VAL;
    hcsr04->echo_begin_ms = HAL_GetTick();
}

/**
 * ECHO 引脚下降沿触发中断.
 *
 * 记录结束时间和 SysTick 寄存器当前值.
 */
void hcsr04_echo_end(Hcsr04Control *hcsr04)
{
    hcsr04->state = HCSR04_ECHO_END;
    hcsr04->echo_end_tick = SysTick->VAL;
    hcsr04->echo_end_ms = HAL_GetTick();
}

uint32_t hcsr04_read_nowait(const Hcsr04Control *hcsr04)
{
    return _read(hcsr04);
}

uint32_t hcsr04_read(const Hcsr04Control *hcsr04)
{
    uint32_t begin = HAL_GetTick();
    while ((hcsr04->state != HCSR04_ECHO_END)
        && (HAL_GetTick() <= begin + HCSR04_TIMEOUT));

    return _read(hcsr04);
}

uint32_t hcsr04_get_dist_async(Hcsr04Control *hcsr04)
{
    uint32_t begin = HAL_GetTick();

    /* 发射超声波 */
    hcsr04_trigger(hcsr04);

    /* 等待 ECHO 信号 */
    while ((hcsr04->state != HCSR04_ECHO_BEGIN)
        && (HAL_GetTick() <= begin + HCSR04_TIMEOUT));

    if (hcsr04->state != HCSR04_ECHO_BEGIN)
    {
        return HCSR04_INVALID_DISTANCE;
    }

    /* 阻塞读测量距离 */
    return hcsr04_read(hcsr04);
}
