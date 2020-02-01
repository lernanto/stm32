/**
 *
 */

#include <string.h>

#include "stm32f1xx_hal.h"

#include "util.h"
#include "sd.h"


/* SD 协议命令 */
#define SD_GO_IDLE_STATE        0x40U   /**< 重置到 IDLE 状态 */
#define SD_SEND_OP_COND         0x41U   /**< 进入 SPI 模式 */
#define SD_SEND_CSD             0x49U   /**< 读取卡特定数据 */
#define SD_SEND_CID             0x4AU   /**< 读取 SD 卡 ID */
#define SD_STOP_TRANSMISSION    0x4CU   /**< 中止读取多块操作 */
#define SD_SEND_STATUS          0x4DU   /**< 读取 SD 卡状态 */
#define SD_SET_BLOCKLEN         0x50U   /**< 设置块长度 */
#define SD_READ_SINGLE_BLOCK    0x51U   /**< 读取单块数据 */
#define SD_READ_MULTIPLE_BLOCK  0x52U   /**< 读取连续的多块数据 */
#define SD_WRITE_BLOCK          0x58U   /**< 写单块数据 */
#define SD_WRITE_MULTIPLE_BLOCK 0x59U   /**< 写连续的多块数据 */
#define SD_PROGRAM_CSD          0x5BU   /**< 写入 CSD */
#define SD_SET_WRITE_PROT       0x5CU   /**< 设置指定地址为写保护 */
#define SD_CLR_WRITE_PROT       0x5DU   /**< 清除写保护 */
#define SD_SEND_WRITE_PROT      0x5EU   /**< 读取写保护状态 */
#define SD_ERASE_WR_BLK_START_ADDR  0x60U   /**< 设置擦除的起始地址 */
#define SD_ERASE_WR_BLK_END_ADDR    0x61U   /**< 设置擦除的结束地址 */
#define SD_ERASE                0x66U   /**< 擦除先前设置的数据块 */

#define SD_GO_IDLE_STATE_CRC    0x95U   /**< GO_IDLE_STATE 命令的 CRC 校验码 */

#define SD_START_BLOCK          0xFEU   /**< 写单块数据时用于开始数据传输 */
#define SD_START_BLOCK_MBW      0xFCU   /**< 写多块数据开始一块数据传输 */
#define SD_STOP_TRAN            0xFDU   /**< 写多块数据结束所有数据块传输 */

#define SD_CMD_LEN      6   /**< SD 命令长度，单位字节 */
#define SD_CMD_RETRY    3   /**< 发送命令/接收响应重试次数 */
#define SD_POLL_RETRY   100 /**< 初始化时等待 SD 卡就绪重试次数 */


/**
 * SD 卡 SPI 片选使能.
 */
__STATIC_INLINE void sd_enable(SdControl *sd)
{
    HAL_GPIO_WritePin(sd->cs_port, sd->cs_pin, GPIO_PIN_RESET);
    delayus(10);
}

/**
 * SD 卡 SPI 取消片选.
 */
__STATIC_INLINE void sd_disable(SdControl *sd)
{
    HAL_GPIO_WritePin(sd->cs_port, sd->cs_pin, GPIO_PIN_SET);
}

/**
 * 向 SD 卡发送命令.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_send(SdControl *sd, uint8_t *cmd)
{
    return HAL_SPI_Transmit(sd->spi, cmd, SD_CMD_LEN, sd->timeout);
}

/**
 * 向 SD 卡发送命令并接收响应.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_send_recv(
    SdControl *sd,
    uint8_t *cmd,
    uint8_t *rsp,
    uint32_t len
)
{
    HAL_StatusTypeDef status;
    size_t i;

    i = 0;
    do
    {
        status = sd_send(sd, cmd);
        ++i;
    } while ((status != HAL_OK) && (i <= SD_CMD_RETRY));

    if (HAL_OK != status)
    {
        return status;
    }

    i = 0;
    do
    {
        status = HAL_SPI_Receive(sd->spi, rsp, len, sd->timeout);
        ++i;
    } while ((rsp[0] & 0x80U) && (i <= SD_CMD_RETRY));

    return status;
}

/**
 * 向 SD 卡发送命令并接收 R1 响应.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_cmd_r1(
    SdControl *sd,
    uint8_t *cmd,
    uint8_t *rsp
)
{
    return sd_send_recv(sd, cmd, rsp, 1);
}

/**
 * 向 SD 卡发送命令并接收 R2 响应.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_cmd_r2(
    SdControl *sd,
    uint8_t *cmd,
    uint16_t *rsp
)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status = sd_send_recv(sd, cmd, buf, ARRAYSIZE(buf));
    
    if (HAL_OK == status)
    {
        *rsp = (((uint16_t)buf[0] << 8) | buf[1]);
    }

    return status;
}

/**
 * 等待 SD 卡 BUSY 信号结束.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_wait(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t rsp = 0;
    size_t i = 0;

    do
    {
        status = HAL_SPI_Receive(sd->spi, &rsp, 1, sd->timeout);
        ++i;
    } while ((HAL_OK == status) && (0 == rsp) && (i <= SD_POLL_RETRY));

    return (0 == rsp) ? 0x80000000U : status;
}

int sd_spi_init(
    SdControl *sd,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint32_t timeout
)
{
    sd->spi = spi;
    sd->cs_port = cs_port;
    sd->cs_pin = cs_pin;
    sd->timeout = timeout;

    return 1;
}

uint32_t sd_power_on(SdControl *sd)
{
    static uint8_t dummy = 0xFFU;

    sd_disable(sd);

    /* SD 卡上电后要持续发送高电平至少74个时钟周期 */
    for (size_t i = 0; i < 74; ++i)
    {
        HAL_SPI_Transmit(sd->spi, &dummy, 1, sd->timeout);
    }

    return 0;
}

uint32_t sd_reset(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t cmd[SD_CMD_LEN];
    uint8_t rsp = 0;
    size_t i = 0;

    /* 发送 GO_IDLE_STATE 命令使 SD 卡进入 IDLE 状态 */
    sd_enable(sd);

    memset(cmd, 0, ARRAYSIZE(cmd));
    cmd[0] = SD_GO_IDLE_STATE;
    cmd[5] = SD_GO_IDLE_STATE_CRC;

    i = 0;
    do
    {
        status = sd_send(sd, cmd);
        ++i;
    } while ((status != HAL_OK) && (i <= SD_CMD_RETRY));

    for (i = 0; i < SD_POLL_RETRY; ++i)
    {
        status = HAL_SPI_Receive(sd->spi, &rsp, 1, sd->timeout);
        if ((HAL_OK == status) && (SD_IN_IDLE_STATE == rsp))
        {
            break;
        }
    }

    sd_disable(sd);

    /* 向 SD 卡发送额外的8个时钟结束通信 */
    cmd[0] = 0xFFU;
    HAL_SPI_Transmit(sd->spi, cmd, 1, sd->timeout);

    if (status != HAL_OK)
    {
        return 0x80000000U | status;
    }

    return rsp;
}

uint32_t sd_init(SdControl *sd)
{
    uint32_t code = 0;
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t cmd[SD_CMD_LEN];
    uint8_t rsp = 0;

    if ((code = sd_power_on(sd)) != 0)
    {
        return code;
    }

    if ((code = sd_reset(sd)) != 0x01U)
    {
        return code;
    }

    /* SD 卡处在 IDLE 状态，不断发送 SEND_OP_COND 命令直到 SD 卡就绪 */
    sd_enable(sd);
    cmd[0] = SD_SEND_OP_COND;
    cmd[5] = 0x01U;
    for (size_t i = 0; i < SD_POLL_RETRY; ++i)
    {
        if (((status = sd_cmd_r1(sd, cmd, &rsp)) == HAL_OK)
            && (0 == rsp))
        {
            break;
        }
    }

    sd_disable(sd);
    /* 不知为什么，初始化最后要提供额外8个时钟 */
    HAL_SPI_Receive(sd->spi, cmd, 1, sd->timeout);

    if (status != HAL_OK)
    {
        return 0x80000000U | status;
    }

    return rsp;
}

uint32_t sd_status(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t cmd[6];
    uint16_t rsp = 0;

    sd_enable(sd);

    memset(cmd, 0, ARRAYSIZE(cmd));
    cmd[0] = SD_SEND_STATUS;
    cmd[5] = 0x01U;

    status = sd_cmd_r2(sd, cmd, &rsp);
    sd_disable(sd);

    if (status != HAL_OK)
    {
        return 0x80000000 | status;
    }

    return rsp;
}

uint32_t sd_read(SdControl *sd, uint32_t addr, uint8_t *buf)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t cmd[6];
    uint8_t rsp = 0;
    uint8_t crc[2];

    sd_enable(sd);

    memset(cmd, 0, ARRAYSIZE(cmd));
    cmd[0] = SD_READ_SINGLE_BLOCK;
    /* 地址高位在前 */
    cmd[1] = (addr >> 24);
    cmd[2] = ((addr >> 16) & 0xFFU);
    cmd[3] = ((addr >> 8) & 0xFFU);
    cmd[4] = (addr & 0xFFU);
    cmd[5] = 0x01U;

    /* 发送读单个数据块命令 */
    if ((status = sd_cmd_r1(sd, cmd, &rsp)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000U | status;
    }
    if (rsp != 0)
    {
        sd_disable(sd);
        return rsp;
    }

    /* 接收数据 */
    if ((status = HAL_SPI_Receive(sd->spi, buf, SD_BLOCK_LEN, sd->timeout)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000U | status;
    }
    /* 接收 CRC 校验数据 */
    if ((status = HAL_SPI_Receive(sd->spi, crc, 2, sd->timeout)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000U | status;
    }

    sd_disable(sd);
    return 0;
}

size_t sd_readmb(SdControl *sd, uint32_t addr, size_t num, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[6];
    uint8_t rsp = 0;

    sd_enable(sd);

    memset(buf, 0, ARRAYSIZE(buf));
    buf[0] = SD_READ_MULTIPLE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    /* 发送读连续多个数据块命令 */
    if ((status = sd_cmd_r1(sd, buf, &rsp)) != HAL_OK)
    {
        sd_disable(sd);
        return 0;
    }
    if (rsp != 0)
    {
        sd_disable(sd);
        return 0;
    }

    /* 每次接收一块数据 */
    for (size_t i = 0; i < num; ++i, data += SD_BLOCK_LEN)
    {
        if ((status = HAL_SPI_Receive(sd->spi, data, SD_BLOCK_LEN, sd->timeout)) != HAL_OK)
        {
            sd_disable(sd);
            return i;
        }
        /* 接收 CRC 校验数据 */
        status = HAL_SPI_Receive(sd->spi, buf, 2, sd->timeout);
    }

    /* 发送停止传输命令 */
    memset(buf, 0, ARRAYSIZE(buf));
    buf[0] = SD_STOP_TRANSMISSION;
    buf[5] = 0x01U;
    status = sd_cmd_r1(sd, buf, &rsp);

    /* 等待 BUSY 信号结束 */
    status = sd_wait(sd);
    sd_disable(sd);
    return num;
}

uint32_t sd_write(SdControl *sd, uint32_t addr, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[6];
    uint8_t rsp = 0;

    sd_enable(sd);

    memset(buf, 0, ARRAYSIZE(buf));
    buf[0] = SD_WRITE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    /* 发送写单个数据块命令 */
    if ((status = sd_cmd_r1(sd, buf, &rsp)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000 | status;
    }
    if (rsp != 0)
    {
        sd_disable(sd);
        return rsp;
    }

    /* 正式数据开始之前要发送一个开始传输令牌 */
    buf[0] = SD_START_BLOCK;
    if ((status = HAL_SPI_Transmit(sd->spi, buf, 1, sd->timeout)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000 | status;
    }
    /* 传输数据 */
    if ((status = HAL_SPI_Transmit(sd->spi, data, SD_BLOCK_LEN, sd->timeout)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000 | status;
    }

    /* 接收响应 */
    if ((status = HAL_SPI_Receive(sd->spi, &rsp, 1, sd->timeout)) != HAL_OK)
    {
        sd_disable(sd);
        return 0x80000000 | status;
    }

    /* 等待 BUSY 信号结束 */
    status = sd_wait(sd);
    sd_disable(sd);
    return rsp;
}

size_t sd_writemb(SdControl *sd, uint32_t addr, size_t num, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[6];
    uint8_t rsp = 0;

    sd_enable(sd);

    memset(buf, 0, ARRAYSIZE(buf));
    buf[0] = SD_WRITE_MULTIPLE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    /* 发送写连续多个数据块命令 */
    if ((status = sd_cmd_r1(sd, buf, &rsp)) != HAL_OK)
    {
        sd_disable(sd);
        return 0;
    }
    if (rsp != 0)
    {
        sd_disable(sd);
        return 0;
    }

    /* 每次发送一个数据块 */
    for (; num > 0; --num, data += SD_BLOCK_LEN)
    {
        /* 每块数据开始之前发送一个多块写特殊的开始传输令牌 */
        buf[0] = SD_START_BLOCK_MBW;
        if ((status = HAL_SPI_Transmit(sd->spi, buf, 1, sd->timeout)) != HAL_OK)
        {
            sd_disable(sd);
            return num;
        }
        /* 传输数据 */
        if ((status = HAL_SPI_Transmit(sd->spi, data, SD_BLOCK_LEN, sd->timeout)) != HAL_OK)
        {
            sd_disable(sd);
            return num;
        }
        /* 接收响应 */
        if ((status = HAL_SPI_Receive(sd->spi, &rsp, 1, sd->timeout)) != HAL_OK)
        {
            sd_disable(sd);
            return num;
        }
        if ((rsp & 0x1FU) == 0x05U)
        {
            sd_disable(sd);
            return num;
        }

        /* 等待 BUSY 信号结束 */
        status = sd_wait(sd);
    }

    /* 最后发送一个停止传输令牌 */
    buf[0] = SD_STOP_TRAN;
    if ((status = HAL_SPI_Transmit(sd->spi, buf, 1, sd->timeout)) != HAL_OK)
    {
        sd_disable(sd);
        return num;
    }

    /* 等待 BUSY 信号结束 */
    status = sd_wait(sd);

    /* TODO: 使用 SEND_NUM_WR_BLOCKS 命令获取成功写入的块数 */
    sd_disable(sd);
    return num;
}
