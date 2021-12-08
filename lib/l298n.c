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
 * @brief 设置电机正负极的脉冲宽度
 *
 * @param l298n 电机控制结构
 * @param pulse1 正极脉冲宽度
 * @param pulse2 负极脉冲宽度
 * @return 状态码
 */
__STATIC_INLINE void set_pulse(L298nControl *l298n, uint32_t pulse1, uint32_t pulse2)
{
    log_debug("set PWM pulse, pulse1 = %lu, pulse2 = %lu", pulse1, pulse2);
    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel1, pulse1);
    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel2, pulse2);
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

    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel1, 0);
    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel2, 0);
    return stop(l298n);
}

L298NState l298n_get_state(L298nControl *l298n)
{
    log_assert(
        (TIM_CHANNEL_STATE_GET(l298n->timer, l298n->channel1)
            != HAL_TIM_CHANNEL_STATE_RESET)
        && (TIM_CHANNEL_STATE_GET(l298n->timer, l298n->channel2)
            != HAL_TIM_CHANNEL_STATE_RESET)
    );

    if (
        (TIM_CHANNEL_STATE_GET(l298n->timer, l298n->channel1)
            == HAL_TIM_CHANNEL_STATE_READY)
        && (TIM_CHANNEL_STATE_GET(l298n->timer, l298n->channel2)
            == HAL_TIM_CHANNEL_STATE_READY)
    )
    {
        /* 正负极通道都处于停止状态，停止 */
        return L298N_STOP;
    }

    if (
        (TIM_CHANNEL_STATE_GET(l298n->timer, l298n->channel1)
            == HAL_TIM_CHANNEL_STATE_BUSY)
        && (__HAL_TIM_GET_COMPARE(l298n->timer, l298n->channel1)
            >= get_max_pulse(l298n))
        && (TIM_CHANNEL_STATE_GET(l298n->timer, l298n->channel2)
            == HAL_TIM_CHANNEL_STATE_BUSY)
        && (__HAL_TIM_GET_COMPARE(l298n->timer, l298n->channel2)
            >= get_max_pulse(l298n))
    )
    {
        /* 正负极通道都处于最大脉冲宽度，刹车 */
        return L298N_BRAKE;
    }

    /* 否则为正常运转 */
    return L298N_START;
}

uint32_t l298n_get_max_speed(L298nControl *l298n)
{
    return get_max_pulse(l298n);
}

int32_t l298n_get_speed(L298nControl *l298n)
{
    return (int32_t)__HAL_TIM_GET_COMPARE(l298n->timer, l298n->channel1)
        - (int32_t)__HAL_TIM_GET_COMPARE(l298n->timer, l298n->channel2);
}

HAL_StatusTypeDef l298n_set_speed(L298nControl *l298n, int32_t speed)
{
    uint32_t pulse1 = ((speed > 0) ? MIN((uint32_t)speed, get_max_pulse(l298n)) : 0);
    uint32_t pulse2 = ((speed < 0) ? MIN((uint32_t)-speed, get_max_pulse(l298n)) : 0);

    log_debug("set speed = %d", speed);

    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel1, pulse1);
    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel2, pulse2);
    return HAL_OK;
}

HAL_StatusTypeDef l298n_brake(L298nControl *l298n)
{
    log_debug("brake, set both channels high");

    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel1, get_max_pulse(l298n));
    __HAL_TIM_SET_COMPARE(l298n->timer, l298n->channel2, get_max_pulse(l298n));
    return l298n_start(l298n);
}
