/**
 * @brief L298N 电机驱动模块驱动程序
 *
 * 通过脉冲宽度调制（PWM）控制电机转速
 */

#ifndef _L298N_H
#define _L298N_H

#include "stm32f1xx_hal.h"


/**
 * @brief 电机状态
 */
typedef enum
{
    L298N_STOP = 0,     /**< 停止 */
    L298N_START,        /**< 已启动 PWM，正常运转 */
    L298N_BRAKE,        /**< 刹车 */
} L298NState;

/**
 * @brief L298N 电机驱动模块控制结构
 *
 * 通过脉冲宽度调制控制直流电机转速
 *  - 当 channel1 和 channel2 电压均为0时，电机停止
 *  - 当 channel1 电压 > 0，channel2 电压 = 0时，电机正转
 *  - 当 channel1 电压 = 0，channel2 电压 > 0时，电机反转
 *  - 当 channel1 和 channel2 电压均为1时，电机锁止
 */
typedef struct
{
    TIM_HandleTypeDef *timer;   /**< 用于 PWM 输出的定时器 */
    uint32_t channel1;
    uint32_t channel2;
} L298nControl;

/**
 * @brief 初始化电机控制结构
 *
 * @param l298n 待初始化的电机控制结构
 * @param timer 用于 PWM 控制电机的定时器
 * @param channel1 正极 PWM 通道
 * @param channel2 负极 PWM 通道
 * @return 成功返回非0，失败返回0
 */
extern int l298n_init(
    L298nControl *l298n,
    TIM_HandleTypeDef *timer,
    uint32_t channel1,
    uint32_t channel2
);

/**
 * @brief 启动电机
 *
 * @param l298n 电机控制结构
 * @return 状态码
 */
extern HAL_StatusTypeDef l298n_start(L298nControl *l298n);

/**
 * @brief 停止电机
 *
 * @param l298n 电机控制结构
 * @return 状态码
 */
extern HAL_StatusTypeDef l298n_stop(L298nControl *l298n);

/**
 * @brief 返回电机当前状态
 *
 * @param l298n 电机控制结构
 * @return 状态码
 */
extern L298NState l298n_get_state(L298nControl *l298n);

/**
 * @brief 返回电机最大脉冲宽度
 *
 * @param l298n 电机控制结构
 * @return 整数表示的脉冲宽度
 */
extern uint32_t l298n_get_max_speed(L298nControl *l298n);

/**
 * @brief 获取电机实际脉冲宽度
 *
 * @param l298n 电机控制结构
 * @return 整数表示的脉冲宽度，正数表示正转，负数表示反转
 */
extern int32_t l298n_get_speed(L298nControl *l298n);

/**
 * @brief 设定驱动电机的脉冲宽度
 *
 * @param l298n 电机控制结构
 * @param speed 脉冲宽度计数，该数除以重载计数即为脉冲占空比。正数表示正转，负数表示反转
 * @return 状态码
 */
extern HAL_StatusTypeDef l298n_set_speed(L298nControl *l298n, int32_t speed);

/**
 * @brief 锁止电机，刹车
 *
 * @param l298n 电机控制结构
 * @return 状态码
 */
extern HAL_StatusTypeDef l298n_brake(L298nControl *l298n);

#endif  /* _L298N_H */
