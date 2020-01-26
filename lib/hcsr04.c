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
    uint16_t echo_pin
)
{
    hcsr04->trig_port = trig_port;
    hcsr04->trig_pin = trig_pin;
    hcsr04->echo_port = echo_port;
    hcsr04->echo_pin = echo_pin;

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

uint32_t hcsr04_get_dist(Hcsr04Control *hcsr04)
{
    uint32_t startms = 0;
    int32_t startval = 0;
    uint32_t endms = 0;
    int32_t endval = 0;
    uint32_t timeus = 0;
    GPIO_PinState echo = GPIO_PIN_RESET;

    /* 发射超声波 */
    hcsr04_trigger(hcsr04);

    /* 等待 ECHO 引脚返回高电平，最多等待1毫秒 */
    startms = HAL_GetTick();
    do
    {
        echo = HAL_GPIO_ReadPin(hcsr04->echo_port, hcsr04->echo_pin);
        endms = HAL_GetTick();
        startval = SysTick->VAL;
    } while ((GPIO_PIN_RESET == echo) && (endms < startms + 1));

    if (GPIO_PIN_RESET == echo)
    {
        /* 超时，检测不到障碍物 */
        return HCSR04_INVALID_DISTANCE;
    }

    /* 检测到 ECHO 高电平，等待高电平结束，计算持续时间 */
    startms = endms;
    while (GPIO_PIN_SET == HAL_GPIO_ReadPin(hcsr04->echo_port, hcsr04->echo_pin));
    endval = SysTick->VAL;
    endms = HAL_GetTick();
    timeus = (endms - startms) * 1000 + (startval - endval) / 72;

    /* 根据超声波在空气中的传播速度计算障碍物距离，单位毫米 */
    return timeus * 170 / 1000;
}
