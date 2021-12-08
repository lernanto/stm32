/*
 * L298N 电机驱动模块驱动.
 */

#include "stm32f1xx_hal.h"

#include "l298n.h"
#include "log.h"
#include "util.h"


/**
 * @brief 返回 PWM 的重载计数，即可以设置的最大脉冲宽度
 *
 * @param l298n 电机控制结构
 * @return 最大脉冲宽度
 */
__STATIC_INLINE uint32_t get_max_pulse(L298nControl *l298n)
{
    return l298n->timer->Init.Period;
}

/**
 * @brief 返回指定通道的当前脉冲宽度
 *
 * @param instance 内部定时器结构
 * @param channel 指定返回脉冲宽度的通道
 * @return 该通道当前设定的脉冲宽度
 */
__STATIC_INLINE uint32_t get_pulse(TIM_TypeDef *instance, uint32_t channel)
{
    switch(channel)
    {
    case TIM_CHANNEL_1:
        return instance->CCR1;
        break;
    case TIM_CHANNEL_2:
        return instance->CCR2;
        break;
    case TIM_CHANNEL_3:
        return instance->CCR3;
        break;
    case TIM_CHANNEL_4:
        return instance->CCR4;
        break;
    default:
        return 0;
        break;
    }
}

/**
 * @brief 设置电机正负极的脉冲宽度
 *
 * @param l298n 电机控制结构
 * @param pulse1 正极脉冲宽度
 * @param pulse2 负极脉冲宽度
 * @return 状态码
 */
static HAL_StatusTypeDef set_pulse(L298nControl *l298n, uint32_t pulse1, uint32_t pulse2)
{
    HAL_StatusTypeDef status = HAL_OK;
    TIM_OC_InitTypeDef oc;

    log_debug("set PWM pulse, pulse1 = %lu, pulse2 = %lu", pulse1, pulse2);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse = pulse1;
    if ((status = HAL_TIM_PWM_ConfigChannel(l298n->timer, &oc, l298n->channel1)) != HAL_OK)
    {
        return status;
    }

    oc.Pulse = pulse2;
    if ((status = HAL_TIM_PWM_ConfigChannel(l298n->timer, &oc, l298n->channel2)) != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief 关闭 PWM
 *
 * @param l298n 电机控制结构
 * @return 状态码
 */
static HAL_StatusTypeDef stop(L298nControl *l298n)
{
    HAL_StatusTypeDef status = HAL_OK;

    if ((status = HAL_TIM_PWM_Stop(l298n->timer, l298n->channel1)) != HAL_OK)
    {
        log_error("cannot stop PWM, channel = %d, status = %d", l298n->channel1, status);
        return status;
    }
    if ((status = HAL_TIM_PWM_Stop(l298n->timer, l298n->channel2)) != HAL_OK)
    {
        log_error("cannot stop PWM, channel = %d, status = %d", l298n->channel2, status);
        return status;
    }

    return HAL_OK;
}

int l298n_init(
    L298nControl *l298n,
    TIM_HandleTypeDef *timer,
    uint32_t channel1,
    uint32_t channel2
)
{
    l298n->timer = timer;
    l298n->channel1 = channel1;
    l298n->channel2 = channel2;

    return 1;
}

HAL_StatusTypeDef l298n_start(L298nControl *l298n)
{
    HAL_StatusTypeDef status = HAL_OK;

    log_debug("start PWM");

    if ((status = HAL_TIM_PWM_Start(l298n->timer, l298n->channel1)) != HAL_OK)
    {
        log_error("cannot start PWM, channel = %d, status = %d", l298n->channel1, status);
        return status;
    }
    if ((status = HAL_TIM_PWM_Start(l298n->timer, l298n->channel2)) != HAL_OK)
    {
        log_error("cannot start PWM, channel = %d, status = %d", l298n->channel2, status);
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef l298n_stop(L298nControl *l298n)
{
    log_debug("stop PWM");

    set_pulse(l298n, 0, 0);
    return stop(l298n);
}

uint32_t l298n_get_max_speed(L298nControl *l298n)
{
    return get_max_pulse(l298n);
}

int32_t l298n_get_speed(L298nControl *l298n)
{
    return (int32_t)get_pulse(l298n->timer->Instance, l298n->channel1)
        - (int32_t)get_pulse(l298n->timer->Instance, l298n->channel2);
}

HAL_StatusTypeDef l298n_set_speed(L298nControl *l298n, int32_t speed)
{
    uint32_t pulse1 = ((speed > 0) ? MIN((uint32_t)speed, get_max_pulse(l298n)) : 0);
    uint32_t pulse2 = ((speed < 0) ? MIN((uint32_t)-speed, get_max_pulse(l298n)) : 0);
    HAL_StatusTypeDef status = HAL_OK;

    log_debug("set speed = %d", speed);

    if ((status = set_pulse(l298n, pulse1, pulse2)) != HAL_OK)
    {
        return status;
    }

    /* TODO: 这里需要自动停吗？停止状态下需不需要自动启动？ */
    if (0 == speed)
    {
        log_debug("speed = 0, stop PWM");
        return stop(l298n);
    }

    return HAL_OK;
}

HAL_StatusTypeDef l298n_brake(L298nControl *l298n)
{
    uint32_t max_pulse = get_max_pulse(l298n);
    HAL_StatusTypeDef status = HAL_OK;

    log_debug("brake, set both channels high");

    if ((status = set_pulse(l298n, max_pulse, max_pulse)) != HAL_OK)
    {
        return status;
    }
    return l298n_start(l298n);
}
