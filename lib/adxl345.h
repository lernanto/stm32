/**
 * ADXL345 加速度传感器驱动.
 * 
 * @see https://www.analog.com/media/cn/technical-documentation/data-sheets/ADXL345_cn.pdf
 */
#ifndef _ADXL345_H
#define _ADXL345_H

#include <stddef.h>

#include "stm32f1xx_hal.h"


/* ADXL345 I2C 设备地址 */
#define ADXL345_ADDR        0x3A    /**< I2C 地址 */
#define ADXL345_ALT_ADDR    0xA6    /**< 备用 I2C 地址 */

/* 地址中的标志位 */
#define ADXL345_RW          0x80    /**< 读写标志位 */
#define ADXL345_MB          0x40    /**< 多字节读写位 */

/* ADXL345 寄存器地址 */
#define ADXL345_DEVID       0x00    /**< 器件 ID */
#define ADXL345_THRESH_TAP  0x1D    /**< 敲击阈值 */
#define ADXL345_OFSX        0x1E    /**< X 轴偏移 */
#define ADXL345_OFSY        0X1F    /**< Y 轴偏移 */
#define ADXL345_OFSZ        0x20    /**< Z 轴偏移 */
#define ADXL345_DUR         0x21    /**< 敲击持续时间 */
#define ADXL345_LATENT      0x22    /**< 敲击延迟 */
#define ADXL345_WINDOW      0x23    /**< 敲击窗口 */
#define ADXL345_THRESH_ACK  0x24    /**< 活动阈值 */
#define ADXL345_THRESH_INACT    0x25    /**< 静止阈值 */
#define ADXL345_TIME_INACT  0x26    /**< 静止时间 */
#define ADXL345_ACT_INACT_CTL   0x27    /**< 轴使能控制活动和静止检测 */
#define ADXL345_THRESH_FF   0x28    /**< 自由落体阈值 */
#define ADXL345_TIME_FF     0x29    /**< 自由落体时间 */
#define ADXL345_TAP_AXES    0x2A    /**< 单击/双击轴控制 */
#define ADXL345_ACT_TAP_STATUS  0x2B    /**< 单击/双击源 */
#define ADXL345_BW_RATE     0x2C    /**< 数据速率及功率模式控制 */
#define ADXL345_POWER_CTL   0x2D    /**< 省电特性控制 */
#define ADXL345_INT_ENABLE  0x2E    /**< 中断使能控制 */
#define ADXL345_INT_MAP     0x2F    /**< 中断映射控制 */
#define ADXL345_INT_SOURCE  0x30    /**< 中断源 */
#define ADXL345_DATA_FORMAT 0x31    /**< 数据格式控制 */
#define ADXL345_DATAX0      0x32    /**< X 轴数据0 */
#define ADXL345_DATAX1      0x33    /**< X 轴数据1 */
#define ADXL345_DATAY0      0x34    /**< Y 轴数据0 */
#define ADXL345_DATAY1      0x35    /**< Y 轴数据1 */
#define ADXL345_DATAZ0      0x36    /**< Z 轴数据0 */
#define ADXL345_DATAZ1      0x37    /**< Z 轴数据1 */
#define ADXL345_FIFO_CTL    0x38    /**< FIFO 控制 */
#define ADXL345_FIFO_STATUS 0x39    /**< FIFO 状态 */

/* 一些常量及标志位 */
#define ADXL345_DEVICE_ID   0xE5    /**< 器件 ID */

/**
 * ADXL345 通信模式，I2C 或 SPI.
 */
typedef enum
{
    ADXL345_I2C,        /**< I2C 模式 */
    ADXL345_SPI,        /**< SPI 模式 */
} Adxl345Mode;

/**
 * ADXL345 控制数据，抽象的基类.
 */
typedef struct
{
    Adxl345Mode mode;       /**< 通信模式 */
    /* 用于兼容多种通信模式的函数指针 */
    /** 写1个字节 */
    HAL_StatusTypeDef (*write)(void *adxl345, uint8_t addr, uint8_t data);
    /** 读1个字节 */
    HAL_StatusTypeDef (*read)(void *adxl345, uint8_t addr, uint8_t *data);
    /** 读2个字节 */
    HAL_StatusTypeDef (*readmb)(void *adxl345, uint8_t addr, uint16_t *data);
} Adxl345Control;

/**
 * ADXL345 I2C 总线控制数据.
 */
typedef struct
{
    /* 头部和基类相同 */
    Adxl345Mode mode;           /**< 通信模式，I2C */
    HAL_StatusTypeDef (*write)(void *adxl345, uint8_t addr, uint8_t data);
    HAL_StatusTypeDef (*read)(void *adxl345, uint8_t addr, uint8_t *data);
    HAL_StatusTypeDef (*readmb)(void *adxl345, uint8_t addr, uint16_t *data);
    I2C_HandleTypeDef *i2c;     /**< 连接的 I2C 总线 */
    uint8_t addr;               /**< I2C 地址 */
    uint32_t timeout;           /**< 超时时间，单位微秒 */
} Adxl345I2cControl;

typedef struct
{
    Adxl345Mode mode;           /**< 通信模式，SPI */
    HAL_StatusTypeDef (*write)(void *adxl345, uint8_t addr, uint8_t data);
    HAL_StatusTypeDef (*read)(void *adxl345, uint8_t addr, uint8_t *data);
    HAL_StatusTypeDef (*readmb)(void *adxl345, uint8_t addr, uint16_t *data);
    SPI_HandleTypeDef *spi;     /**< 连接的 SPI 总线 */
    GPIO_TypeDef *cs_port;      /**< CS 使能信号端口 */
    uint16_t cs_pin;            /**< CS 使能信号引脚 */
    uint32_t timeout;           /**< 超时等待时间 */
} Adxl345SpiControl;

/**
 * 初始化 ADXL345 I2C 模式.
 */
extern HAL_StatusTypeDef adxl345_i2c_init(
    Adxl345I2cControl *adxl345,
    I2C_HandleTypeDef *i2c,
    uint8_t addr,
    uint32_t timeout
);

/**
 * 向指定地址寄存器写入1字节数据.
 */
extern HAL_StatusTypeDef adxl345_i2c_write(
    Adxl345I2cControl *adxl345,
    uint8_t addr,
    uint8_t data
);

/**
 * 从指定地址寄存器读取1字节数据.
 */
extern HAL_StatusTypeDef adxl345_i2c_read(
    Adxl345I2cControl *adxl345,
    uint8_t addr,
    uint8_t *data
);

/**
 * 从指定地址开始连续读取2字节数据.
 */
extern HAL_StatusTypeDef adxl345_i2c_readmb(
    Adxl345I2cControl *adxl345,
    uint8_t addr,
    uint16_t *data
);

/**
 * 初始化 ADXL345 SPI 模式.
 */
extern HAL_StatusTypeDef adxl345_spi_init(
    Adxl345SpiControl *adxl345,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint32_t timeout
);

/**
 * 向指定地址寄存器写入1字节数据.
 */
extern HAL_StatusTypeDef adxl345_spi_write(
    Adxl345SpiControl *adxl345,
    uint8_t addr,
    uint8_t data
);

/**
 * 从指定地址寄存器读取1字节数据.
 */
extern HAL_StatusTypeDef adxl345_spi_read(
    Adxl345SpiControl *adxl345,
    uint8_t addr,
    uint8_t *data
);

/**
 * 从指定地址开始连续读取2字节数据.
 */
extern HAL_StatusTypeDef adxl345_spi_readmb(
    Adxl345SpiControl *adxl345,
    uint8_t addr,
    uint16_t *data
);

/**
 * 初始化 ADXL345.
 */
extern HAL_StatusTypeDef adxl345_init(Adxl345Control *adxl345);

/**
 * 读取3个方向的加速度.
 */
extern HAL_StatusTypeDef adxl345_get_acc(
    Adxl345Control *adxl345,
    int32_t *x,
    int32_t *y,
    int32_t *z
);

#endif  /* _ADXL345_H */
