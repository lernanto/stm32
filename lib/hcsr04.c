/**
 * HC-SR04 超声测距传感器驱动.
 */

#include "stm32f1xx_hal.h"

#include "util.h"
#include "hcsr04.h"

int hcsr04_init(
    Hcsr04Control *hcsr04,
    GPIO_TypeDef *trig_port,
    uint16_t trig_pin,
    GPIO_TypeDef *echo_port,
    uint16_t echo_pin,
    uint32_t timeout
)
{
    hcsr04->trig_port = trig_port;
    hcsr04->trig_pin = trig_pin;
    hcsr04->echo_port = echo_port;
    hcsr04->echo_pin = echo_pin;
    hcsr04->timeout = timeout;
    hcsr04->state = HCSR04_RESET;
    hcsr04->echo_begin_ms = 0;
    hcsr04->echo_begin_tick = 0;
    hcsr04->echo_end_ms = 0;
    hcsr04->echo_end_tick = 0;

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
    uint32_t timeus = 0;
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
    } while ((GPIO_PIN_RESET == echo) && (end_ms <= begin_ms + hcsr04->timeout));

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

    /* ECHO 高电平持续时间 */
    timeus = (end_ms - begin_ms) * 1000
        + ((int32_t)begin_tick - (int32_t)end_tick) / 72;

    /* 根据超声波在空气中的传播速度计算障碍物距离，单位毫米 */
    return timeus * 170 / 1000;
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

uint32_t hcsr04_read(Hcsr04Control *hcsr04)
{
    uint32_t timeus = 0;

    if (hcsr04->timeout > 0)
    {
        /* 设置了超时时间，尝试等待 ECHO 信号结束 */
        uint32_t begin = HAL_GetTick();
        while ((hcsr04->state != HCSR04_ECHO_END)
            && (HAL_GetTick() <= begin + hcsr04->timeout));
    }

    /* 未接收到 ECHO 信号或信号未结束 */
    if ((hcsr04->state != HCSR04_ECHO_END)
        /* 距离上次接收到 ECHO 信号已超过指定时间 */
        || (HAL_GetTick() > hcsr04->echo_end_ms + hcsr04->timeout))
    {
        return HCSR04_INVALID_DISTANCE;
    }

    /* 根据记录的 ECHO 开始和结束时间计算持续时间，前者是中断设置的 */
    timeus = (hcsr04->echo_end_ms - hcsr04->echo_begin_ms) * 1000
        + ((int32_t)hcsr04->echo_begin_tick - (int32_t)hcsr04->echo_end_tick) / 72;

    /* 根据超声波在空气中的传播速度计算障碍物距离，单位毫米 */
    return timeus * 170 / 1000;
}

uint32_t hcsr04_get_dist(Hcsr04Control *hcsr04)
{
    uint32_t timeus = 0;
    uint32_t begin_tick = SysTick->VAL;
    uint32_t begin_ms = HAL_GetTick();

    /* 发射超声波 */
    hcsr04_trigger(hcsr04);

    /* 等待 ECHO 信号结束，结束时间等由中断设置 */
    while (((hcsr04->echo_end_ms < begin_ms)
        || ((hcsr04->echo_end_ms == begin_ms) && (hcsr04->echo_end_tick >= begin_tick)))
        && (HAL_GetTick() <= begin_ms + hcsr04->timeout + HCSR04_TIMEOUT));

    if ((hcsr04->echo_end_ms < begin_ms)
        || ((hcsr04->echo_end_ms == begin_ms) && (hcsr04->echo_end_tick >= begin_tick)))
    {
        return HCSR04_INVALID_DISTANCE;
    }

    timeus = (hcsr04->echo_end_ms - hcsr04->echo_begin_ms) * 1000
        + ((int32_t)hcsr04->echo_begin_tick - (int32_t)hcsr04->echo_end_tick) / 72;

    return timeus * 170 / 1000;
}

