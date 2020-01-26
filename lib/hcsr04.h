/**
 * HC-SR04 超声测距传感器驱动.
 */

#ifndef _HCSR04_H
#define _HCSR04_H

#include <stddef.h>

#include "stm32f1xx_hal.h"

/** 检测不到障碍物时返回的无效距离 */
#define HCSR04_INVALID_DISTANCE UINT32_MAX

typedef struct
{
    GPIO_TypeDef *trig_port;    /**< TRIG 端口 */
    uint16_t trig_pin;          /**< TRIG 引脚 */
    GPIO_TypeDef *echo_port;    /**< ECHO 端口 */
    uint16_t echo_pin;          /**< ECHO 引脚 */
} Hcsr04Control;

/**
 * 初始化接线.
 */
extern int hcsr04_init(
    Hcsr04Control *hcsr04,
    GPIO_TypeDef *trig_port,
    uint16_t trig_pin,
    GPIO_TypeDef *echo_port,
    uint16_t echo_pin
);

/**
 * 触发一次测距，发射一组超声波.
 */
extern int hcsr04_trigger(Hcsr04Control *hcsr04);

/**
 * 获取前方障碍物距离.
 */
extern uint32_t hcsr04_get_dist(Hcsr04Control *hcsr04);

#endif  /* _HCSR04_H */
