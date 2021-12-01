/**
 * HC-SR04 超声测距传感器驱动.
 */

#ifndef _HCSR04_H
#define _HCSR04_H

#include <stddef.h>

#include "stm32f1xx_hal.h"


/** 超声波在空气中的传播速度，单位 mm/ms */
#define HCSR04_ULTRASOUND_SPEED 340

/** HC-SR04 的最小量程，单位毫米 */
#define HCSR04_MIN_DISTANCE 20

/** HC-SR04 的最大量程，单位毫米 */
#define HCSR04_MAX_DISTANCE 4000

/** 等待 ECHO 结束的超时时间，单位毫秒，根据 HC-SR04 量程估算得出 */
#define HCSR04_TIMEOUT  (HCSR04_MAX_DISTANCE * 2 / HCSR04_ULTRASOUND_SPEED + 2)

/** 检测不到障碍物时返回的无效距离 */
#define HCSR04_INVALID_DISTANCE UINT32_MAX

/**
 * HC-SR04 状态.
 */
typedef enum
{
    HCSR04_RESET = 0,   /**< 初始化完成，未执行任何操作 */
    HCSR04_TRIGGER,     /**< 已发送超声波，但未接收到 ECHO 信号 */
    HCSR04_ECHO_BEGIN,  /**< 已接收到 ECHO 信号，但信号未结束 */
    HCSR04_ECHO_END,
} Hcsr04State;

/**
 * HC-SR04 控制数据结构.
 */
typedef struct
{
    GPIO_TypeDef *trig_port;            /**< TRIG 端口 */
    uint16_t trig_pin;                  /**< TRIG 引脚 */
    GPIO_TypeDef *echo_port;            /**< ECHO 端口 */
    uint16_t echo_pin;                  /**< ECHO 引脚 */
    uint32_t expire;                    /**< 测量结果过期时间 */
    volatile Hcsr04State state;         /**< 传感器当前状态 */
    volatile uint32_t echo_begin_ms;    /**< ECHO 引脚高电平开始时间，毫秒 */
    volatile uint32_t echo_begin_tick;  /**< ECHO 引脚高电平开始时 SysTick 寄存器值 */
    volatile uint32_t echo_end_ms;      /**< ECHO 引脚高电平结束时间，毫秒 */
    volatile uint32_t echo_end_tick;    /**< ECHO 引脚高电平结束时 SysTick 寄存器值 */
} Hcsr04Control;

/**
 * 初始化数据结构.
 */
extern int hcsr04_init(
    Hcsr04Control *hcsr04,
    GPIO_TypeDef *trig_port,
    uint16_t trig_pin,
    GPIO_TypeDef *echo_port,
    uint16_t echo_pin,
    uint32_t expire
);

/**
 * 触发一次测距，发射一组超声波.
 */
extern int hcsr04_trigger(Hcsr04Control *hcsr04);

/**
 * 同步测量障碍物距离.
 *
 * 发射超声波，然后阻塞等待 ECHO 信号结束，返回距离
 */
extern uint32_t hcsr04_get_dist_sync(Hcsr04Control *hcsr04);

/**
 * ECHO 上升沿触发中断，记录 ECHO 信号开始时间.
 */
extern void hcsr04_echo_begin(Hcsr04Control *hcsr04);

/**
 * ECHO 下降沿触发中断，记录 ECHO 信号结束时间.
 */
extern void hcsr04_echo_end(Hcsr04Control *hcsr04);

/**
 * 异步获取前方障碍物距离.
 *
 * 不发射超声波，也不检测 ECHO 信号，只根据中断记录的 ECHO 时间计算上次测量的距离.
 */
extern uint32_t hcsr04_read_nowait(const Hcsr04Control *hcsr04);

/**
 * 获取障碍物的距离，如果正在接收 ECHO 信号，等待信号结束再计算.
 */
extern uint32_t hcsr04_read(const Hcsr04Control *hcsr04);

/**
 * 发射超声波并获取距离.
 *
 * 保证获取的距离是在触发之后测得的.
 */
extern uint32_t hcsr04_get_dist_async(Hcsr04Control *hcsr04);

#endif  /* _HCSR04_H */
