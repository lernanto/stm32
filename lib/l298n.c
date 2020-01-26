/**
 * L298N 电机驱动模块驱动.
 */

#include "stm32f1xx_hal.h"

#include "l298n.h"


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

HAL_StatusTypeDef l298n_set_speed(
    L298nControl *l298n,
    int32_t speed
)
{
    HAL_StatusTypeDef status = HAL_OK;
    TIM_OC_InitTypeDef oc;
    uint32_t pulse1 = ((speed > 0) ? speed : 0);
    uint32_t pulse2 = ((speed < 0) ? -speed : 0);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse = pulse1;
    if ((status = HAL_TIM_OC_ConfigChannel(l298n->timer, &oc, l298n->channel1)) != HAL_OK)
    {
        return status;
    }

    oc.Pulse = pulse2;
    if ((status = HAL_TIM_OC_ConfigChannel(l298n->timer, &oc, l298n->channel2)) != HAL_OK)
    {
        return status;
    }

    if ((status = HAL_TIM_PWM_Start(l298n->timer, l298n->channel1)) != HAL_OK)
    {
        return status;
    }
    if ((status = HAL_TIM_PWM_Start(l298n->timer, l298n->channel2)) != HAL_OK)
    {
        return status;
    }

    return status;
}
