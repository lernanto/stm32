/**
 * L298N 电机驱动模块驱动程序.
 * 
 * 通过脉冲宽度调制（PWM）控制电机转速.
 */

#ifndef _L298N_H
#define _L298N_H

#include "stm32f1xx_hal.h"


/**
 * L298N 电机驱动模块控制结构.
 * 
 * 通过脉冲宽度调制控制电机转速
 *  - 当 channel1 和 channel2 电压均为0时，电机停止
 *  - 当 channel1 电压 > 0，channel2 电压 = 0时，电机正转
 *  - 当 channel1 电压 = 0，channel2 电压 > 0时，电机反转
 */
typedef struct
{
    TIM_HandleTypeDef *timer;   /**< 用于 PWM 输出的定时器 */
    uint32_t channel1;
    uint32_t channel2;
} L298nControl;

extern int l298n_init(
    L298nControl *l298n,
    TIM_HandleTypeDef *timer,
    uint32_t channel1,
    uint32_t channel2
);

extern HAL_StatusTypeDef l298n_set_speed(
    L298nControl *l298n,
    int32_t speed
);

#endif  /* _L298N_H */
