/**
 * SD 卡驱动.
 *
 * 当前只支持 SPI 模式.
 */

#ifndef _SD_H
#define _SD_H

#include "stm32f1xx_hal.h"


/* SD 应答错误码 */
#define SD_IN_IDLE_STATE    0x01U
#define SD_ERASE_RESET      0x02U
#define SD_ILLEGAL_COMMAND  0x04U
#define SD_COM_CRC_ERROR    0x08U
#define SD_ERASE_SEQ_ERROR  0x10U
#define SD_ADDRESS_ERROR    0x20U
#define SD_PARAMETER_ERROR  0x40U

#define SD_CARD_IS_LOCKED   0x01U
#define SD_WP_ERASE_SKIP    0x02U
#define SD_LOCK_UNLOCK_CMD_FAILED   0x02U
#define SD_ERROR            0x04U
#define SD_CC_ERROR         0x08U
#define SD_CARD_ECC_FAILED  0x10U
#define SD_WP_VIOLATION     0x20U
#define SD_ERASE_PARAM      0x40U
#define SD_OUT_OF_RANGE     0x80U
#define SD_CSD_OVERWRITE    0x80U

#define SD_BLOCK_LEN    512     /**< 默认块长度，单位字节 */

/**
 * SD 卡控制数据.
 */
typedef struct
{
    SPI_HandleTypeDef *spi;     /**< 连接 SD 卡的 SPI 总线 */
    GPIO_TypeDef *cs_port;      /**< SPI CS 使能端口 */
    uint16_t cs_pin;            /**< SPI CS 引脚 */
    uint32_t timeout;           /**< SPI 超时时间 */
} SdControl;

/**
 * 初始化 SD 卡.
 */
extern int sd_init(
    SdControl *sd,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint32_t timeout
);

/**
 * 重置 SD 卡为 SPI 模式.
 * 
 * @return  成功返回0，失败返回错误码
 *              - SPI 通信错误的情况下，返回的错误码是 HAL_StatusTypeDef 的错误码值
 *              - SD 卡错误的情况下，返回的错误码是 0x80000000 | SD 卡状态码
 */
extern uint32_t sd_reset(SdControl *sd);

/**
 * 从 SD 卡读取1块数据.
 */
extern uint32_t sd_read(SdControl *sd, uint32_t addr, uint8_t *buf);

/**
 * 从 SD 卡读取连续多块数据.
 */
extern uint32_t sd_readmb(SdControl *sd, uint32_t addr, size_t num, uint8_t *buf);

/**
 * 向 SD 卡写入1块数据.
 */
extern uint32_t sd_write(SdControl *sd, uint32_t addr, const uint8_t *buf);

/**
 * 向 SD 卡连续多块写入数据.
 */
extern uint32_t sd_writemb(SdControl *sd, uint32_t addr, size_t num, const uint8_t *buf);

#endif  /* _SD_H */
