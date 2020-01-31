/**
 * SD 卡驱动.
 *
 * @see https://www.convict.lu/pdf/ProdManualSDCardv1.9.pdf
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
#define SD_APP_CMD              0x77U   /**< 通知 SD 卡下一命令是应用特有命令 */
#define SD_GEN_CMD              0x78U
#define SD_READ_OCR             0x7AU   /**< 读取 OCR 寄存器 */
#define SD_CRC_ON_OFF           0x7BU   /**< CRC 校验开关 */

/* 应用特有命令，以下命令必须以 APP_CMD 命令开头 */
#define SD_SD_STATUS            0x4DU   /**< 读取 SD 卡状态 */
#define SD_SEND_NUM_WR_BLOCKS   0x56U   /**< 返回上次写操作成功写入的块数 */
#define SD_SET_WR_BLK_ERASE_COUNT   0x57U   /**< 设置要预擦除的块数，用于快速多块写命令 */
#define SD_SEND_OP_COND2        0x69U   /**< 初始化 SD 卡 */
#define SD_SET_CLR_CARD_DETECT  0x6AU
#define SD_SEND_SCR             0x73U   /**< 读取 SD 卡配置寄存器 */

#define SD_GO_IDLE_STATE_CRC    0x95U   /**< GO_IDLE_STATE 命令的 CRC 校验码 */

#define SD_START_BLOCK          0xFEU   /**< 写单块数据时用于开始数据传输 */
#define SD_START_BLOCK_MBW      0xFCU   /**< 写多块数据开始一块数据传输 */
#define SD_STOP_TRAN            0xFDU   /**< 写多块数据结束所有数据块传输 */

#define SD_CMD_LEN      6   /**< SD 命令长度，单位字节 */
#define SD_SPI_RETRY    3   /**< SPI 发送/接收最大重试次数 */
#define SD_RECV_RETRY   10  /**< 接收响应重试次数，参考手册，应大于 NCR 的最大值 */
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
    static const uint8_t dummy[] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU };

    HAL_GPIO_WritePin(sd->cs_port, sd->cs_pin, GPIO_PIN_SET);
    delayus(10);

    /* 多发送几个时钟使 SD 卡稳定 */
    HAL_SPI_Transmit(sd->spi, (uint8_t *)dummy, ARRAYSIZE(dummy), sd->timeout);
}

/**
 * 等待 SPI 就绪.
 */
__STATIC_INLINE HAL_SPI_StateTypeDef sd_wait_spi(SdControl *sd)
{
    HAL_SPI_StateTypeDef state;
    uint32_t startms = HAL_GetTick();
    uint32_t endms;

    do
    {
        state = HAL_SPI_GetState(sd->spi);
        endms = HAL_GetTick();
    } while ((state != HAL_SPI_STATE_READY) && (endms <= startms + sd->timeout));

    return state;
}

/**
 * 向 SD 卡发送1个字节数据.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_send(SdControl *sd, uint8_t data)
{
    HAL_StatusTypeDef status;
    size_t i;

    if (sd_wait_spi(sd) != HAL_SPI_STATE_READY)
    {
        return HAL_BUSY;
    }

    i = 0;
    do
    {
        status = HAL_SPI_Transmit(sd->spi, &data, 1, sd->timeout);
        ++i;
    } while ((status != HAL_OK) && (i < SD_SPI_RETRY));

    return status;
}

/**
 * 向 SD 卡发送命令.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_cmd(SdControl *sd, uint8_t const *cmd)
{
    HAL_StatusTypeDef status;
    size_t i;

    if (sd_wait_spi(sd) != HAL_SPI_STATE_READY)
    {
        return HAL_BUSY;
    }

    i = 0;
    do
    {
        status = HAL_SPI_Transmit(sd->spi, (uint8_t *)cmd, SD_CMD_LEN, sd->timeout);
        ++i;
    } while ((status != HAL_OK) && (i < SD_SPI_RETRY));

    return status;
}

/**
 * 从 SD 卡接收响应.
 *
 * 命令发出后需要经过一定时钟周期 SD 才会响应，在此之前 SD 卡一直返回高电平
 * 忽略高电平直到接收到有效响应数据
 */
__STATIC_INLINE HAL_StatusTypeDef sd_recv(SdControl *sd, uint8_t *data)
{
    static uint8_t dummy = 0xFFU;
    HAL_StatusTypeDef status;
    size_t i = 0;

    do
    {
        size_t j = 0;

        do
        {
            if (sd_wait_spi(sd) == HAL_SPI_STATE_READY)
            {
                /* 手册显示接收来自 SD 卡的数据时主机应输出高电平 */
                status = HAL_SPI_TransmitReceive(sd->spi, &dummy, data, 1, sd->timeout);
            }
            else
            {
                status = HAL_BUSY;
            }

            ++j;
        } while ((status != HAL_OK) && (j < SD_SPI_RETRY));

        ++i;
    } while ((HAL_OK == status) && (0xFFU == *data) && (i < SD_RECV_RETRY));

    return (i >= SD_RECV_RETRY) ? SD_EXCEED_MAX_RETRY : status;
}

/**
 * 向 SD 卡发送命令并接收 R1 响应.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_cmd_r1(SdControl *sd, const uint8_t *cmd)
{
    uint8_t rsp = 0;
    HAL_StatusTypeDef status = sd_cmd(sd, cmd);
    if (HAL_OK == status)
    {
        status = sd_recv(sd, &rsp);
    }
    sd->status = (HAL_OK == status) ? rsp : (0x80000000U | status);
    return status;
}

/**
 * 向 SD 卡发送命令并接收 R2 响应.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_cmd_r2(SdControl *sd, const uint8_t *cmd)
{
    uint8_t buf = 0;
    HAL_StatusTypeDef status = sd_cmd(sd, cmd);
    if (HAL_OK == status)
    {
        if ((status = sd_recv(sd, &buf)) == HAL_OK)
        {
            sd->status = ((uint32_t)buf << 8);
            status = HAL_SPI_Receive(sd->spi, &buf, 1, sd->timeout);
            sd->status |= buf;
        }
    }

    if (status != HAL_OK)
    {
        sd->status = (0x80000000U | status);
    }

    return status;
}

/**
 * 接收来自 SD 卡的数据块.
 */
__STATIC_INLINE HAL_StatusTypeDef sd_recv_data(
    SdControl *sd,
    uint8_t *data,
    size_t len
)
{
    static uint8_t dummy = 0xFFU;
    uint8_t buf[2];
    HAL_StatusTypeDef status;
    uint32_t startms = HAL_GetTick();
    uint32_t endms;

    /* 等待数据块 */
    do
    {
        if ((sd_wait_spi(sd)) == HAL_SPI_STATE_READY)
        {
            status = HAL_SPI_TransmitReceive(sd->spi, &dummy, buf, 1, sd->timeout);
            endms = HAL_GetTick();
        }
        else
        {
            status = HAL_BUSY;
        }
    } while ((HAL_OK == status) && (0xFFU == buf[0])
        && (endms <= startms + sd->timeout));

    if (status != HAL_OK)
    {
        sd->status = (0x80000000U | status);
        return status;
    }
    if (0xFFU == buf[0])
    {
        sd->status = (0x80000000U | SD_TIMEOUT);
        return SD_TIMEOUT;
    }
    if (buf[0] != SD_START_BLOCK)
    {
        /* 接收到错误响应 */
        sd->status = buf[0];
        return status;
    }

    /* 接收正式数据 */
    status = HAL_SPI_Receive(sd->spi, data, len, sd->timeout);
    /* 接收 CRC 校验数据 */
    HAL_SPI_Receive(sd->spi, buf, 2, sd->timeout);

    if (status != HAL_OK)
    {
        sd->status = (0x80000000U | status);
    }
    return status;
}

/**
 * 等待 SD 卡 BUSY 信号结束.
 *
 * @note SD 卡在 BUSY 状态下输出低电平，BUSY 结束后输出高电平。
 * 但由于 CS 选择 SD 卡后输出有1个时钟的延迟，应该检测 0xFF 而不是 0x00 来确定 SD 已就绪。
 */
__STATIC_INLINE HAL_StatusTypeDef sd_wait_busy(SdControl *sd)
{
    static uint8_t dummy = 0xFFU;
    HAL_StatusTypeDef status;
    uint8_t buf;
    uint32_t startms = HAL_GetTick();
    uint32_t endms;

    do
    {
        size_t i = 0;
        do
        {
            status = HAL_SPI_TransmitReceive(sd->spi, &dummy, &buf, 1, sd->timeout);
            ++i;
        } while ((status != HAL_OK) && (i < SD_SPI_RETRY));
        endms = HAL_GetTick();
    } while ((HAL_OK == status) && (buf != 0xFFU)
        && (endms <= startms + sd->timeout));

    return (HAL_OK == status) ? ((0xFFU == buf) ? HAL_OK : SD_TIMEOUT) : status;
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
    sd->status = 0;

    return 1;
}

uint32_t sd_power_on(SdControl *sd)
{
    sd_disable(sd);

    /* SD 卡上电后要持续发送高电平至少74个时钟周期 */
    for (size_t i = 0; i < 10; ++i)
    {
        sd_send(sd, 0xFFU);
    }

    return 0;
}

uint32_t sd_reset(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];
    size_t i;

    /* 发送 GO_IDLE_STATE 命令使 SD 卡进入 IDLE 状态 */
    sd_enable(sd);
    /* 保险起见也先发送几个时钟 */
    for (i = 0; i < 10; ++i)
    {
        sd_send(sd, 0xFFU);
    }

    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_GO_IDLE_STATE;
    buf[5] = SD_GO_IDLE_STATE_CRC;

    if ((status = sd_cmd(sd, buf)) == HAL_OK)
    {
        /* 反复检测 SD 卡返回值直到 SD 卡进入 IDLE 状态 */
        for (i = 0; i < SD_POLL_RETRY; ++i)
        {
            status = HAL_SPI_Receive(sd->spi, buf, 1, sd->timeout);
            if ((HAL_OK == status) && (SD_IN_IDLE_STATE == buf[0]))
            {
                break;
            }
        }
    }

    sd_disable(sd);
    /* 向 SD 卡发送额外的8个时钟结束通信 */
    sd_send(sd, 0xFFU);

    sd->status = (HAL_OK == status) ? buf[0] : (0x80000000U | status);
    return sd_error(sd);
}

static uint32_t sd_poll(SdControl *sd)
{
    uint8_t cmd[SD_CMD_LEN];
    size_t i;

    /* SD 卡处在 IDLE 状态，不断发送 SEND_OP_COND 命令直到 SD 卡就绪 */
    sd_enable(sd);
    cmd[0] = SD_SEND_OP_COND;
    cmd[5] = 0x01U;
    i = 0;
    do
    {
        sd_cmd_r1(sd, cmd);
        ++i;
    } while ((sd_error(sd) != 0) && (i < SD_POLL_RETRY));

    sd_disable(sd);
    sd_send(sd, 0xFFU);

    return sd_error(sd);
}

uint32_t sd_init(SdControl *sd)
{
    sd_power_on(sd);

    /* 重新初始化也可能调用此函数，保险起见手动重置 SD 卡 */
    if (sd_reset(sd) == SD_IN_IDLE_STATE)
    {
        sd_poll(sd);
    }

    return sd_error(sd);
}

uint32_t sd_reinit(SdControl *sd)
{
    /* 重置 SD 卡使进入 IDLE 状态 */
    if (sd_reset(sd) == SD_IN_IDLE_STATE)
    {
        sd_poll(sd);
    }

    return sd_error(sd);
}

uint32_t sd_ocr(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];
    uint32_t ocr = 0;

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd_error(sd);
    }

    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_READ_OCR;
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (0 == sd_error(sd))
    {
        /* 接收 OCR 寄存器值 */
        if ((status = HAL_SPI_Receive(sd->spi, buf, 4, sd->timeout)) == HAL_OK)
        {
            ocr = (((uint32_t)buf[0] << 24)
                | ((uint32_t)buf[1] << 16)
                | ((uint32_t)buf[2] << 8)
                | buf[3]);
        }
    }

    sd_disable(sd);
    sd_send(sd, 0xFFU);
    return ocr;
}

uint32_t sd_status(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t cmd[SD_CMD_LEN];

    sd_enable(sd);

    /* SD 卡可能处在 BUSY 状态，等待 SD 卡就绪 */
    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd->status;
    }

    memset(cmd, 0, SD_CMD_LEN);
    cmd[0] = SD_SEND_STATUS;
    cmd[5] = 0x01U;
    sd_cmd_r2(sd, cmd);

    sd_disable(sd);
    sd_send(sd, 0xFFU);
    return sd_error(sd);
}

uint32_t sd_read(SdControl *sd, uint32_t addr, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd_error(sd);
    }

    /* 发送读单个数据块命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_READ_SINGLE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (0 == sd_error(sd))
    {
        /* 接收数据块 */
        sd_recv_data(sd, data, SD_BLOCK_LEN);
    }

    sd_disable(sd);
    sd_send(sd, 0xFFU);
    return sd_error(sd);
}

size_t sd_readmb(SdControl *sd, uint32_t addr, size_t num, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd_error(sd);
    }

    /* 发送读连续多个数据块命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_READ_MULTIPLE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (sd_error(sd) != 0)
    {
        sd_disable(sd);
        sd_send(sd, 0xFFU);
        return 0;
    }

    for (size_t i = 0; i < num; ++i, data += SD_BLOCK_LEN)
    {
        /* 每次接收一块数据 */
        sd_recv_data(sd, data, SD_BLOCK_LEN);
        if (sd_error(sd) != 0)
        {
            sd_disable(sd);
            sd_send(sd, 0xFFU);
            return i;
        }
    }

    /* 发送停止传输命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_STOP_TRANSMISSION;
    buf[5] = 0x01U;
    sd_cmd_r1(sd, buf);

    sd_disable(sd);
    sd_send(sd, 0xFFU);
    return num;
}

uint32_t sd_write(SdControl *sd, uint32_t addr, const uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd_error(sd);
    }

    /* 发送写单个数据块命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_WRITE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (sd_error(sd) != 0)
    {
        sd_disable(sd);
        sd_send(sd, 0xFFU);
        return sd_error(sd);
    }

    /* 数据传输前要延迟8个时钟 */
    sd_send(sd, 0xFFU);

    /* 正式数据开始之前要发送一个开始传输令牌 */
    buf[0] = SD_START_BLOCK;
    status = HAL_SPI_Transmit(sd->spi, buf, 1, sd->timeout);
    if (HAL_OK == status)
    {
        /* 传输数据 */
        status = HAL_SPI_Transmit(sd->spi, (uint8_t *)data, SD_BLOCK_LEN, sd->timeout);
        if (HAL_OK == status)
        {
            /* 接收响应 */
            status = sd_recv(sd, buf);
        }
    }

    sd_disable(sd);
    sd_send(sd, 0xFFU);

    if (status != HAL_OK)
    {
        sd->status = (0x80000000U | status);
    }
    else if ((buf[0] & SD_DATA_RESP_MASK) != SD_DATA_ACCEPTED)
    {
        /* 接收到错误响应，使用 SEND_STATUS 命令获取 SD 卡状态 */
        sd_status(sd);
    }

    return sd_error(sd);
}

size_t sd_writemb(SdControl *sd, uint32_t addr, size_t num, const uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];
    uint8_t resp = 0;
    size_t i = 0;

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd_error(sd);
    }

    /* 发送写连续多个数据块命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_WRITE_MULTIPLE_BLOCK;
    /* 地址高位在前 */
    buf[1] = (addr >> 24);
    buf[2] = ((addr >> 16) & 0xFFU);
    buf[3] = ((addr >> 8) & 0xFFU);
    buf[4] = (addr & 0xFFU);
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (sd_error(sd) != 0)
    {
        sd_disable(sd);
        sd_send(sd, 0xFFU);
        return 0;
    }

    /* 每次发送一个数据块 */
    for (i = 0; i < num; ++i, data += SD_BLOCK_LEN)
    {
        /* 延迟8个时钟 */
        sd_send(sd, 0xFFU);

        /* 每块数据开始之前发送一个写多块专用的开始传输令牌 */
        buf[0] = SD_START_BLOCK_MBW;
        if ((status = HAL_SPI_Transmit(sd->spi, buf, 1, sd->timeout)) == HAL_OK)
        {
            /* 传输数据 */
            if ((status = HAL_SPI_Transmit(
                sd->spi,
                (uint8_t *)data,
                SD_BLOCK_LEN,
                sd->timeout
            )) == HAL_OK)
            {
                /* 接收响应 */
                if ((status = sd_recv(sd, &resp)) == HAL_OK)
                {
                    /* 等待 BUSY 信号结束 */
                    status = sd_wait_busy(sd);
                }
            }
        }

        if ((status != HAL_OK)
            || ((resp & SD_DATA_RESP_MASK) != SD_DATA_ACCEPTED))
        {
            break;
        }
    }

    /* 最后发送一个停止传输令牌 */
    if (HAL_OK == status)
    {
        sd_send(sd, 0xFFU);
        buf[0] = SD_STOP_TRAN;
        status = HAL_SPI_Transmit(sd->spi, buf, 1, sd->timeout);
    }

    sd_disable(sd);
    sd_send(sd, 0xFFU);

    if (status != HAL_OK)
    {
        sd->status = (0x80000000U | status);
    }
    else if ((resp & SD_DATA_RESP_MASK) != SD_DATA_ACCEPTED)
    {
        sd_status(sd);
    }

    return i;
}

size_t sd_get_write_num(SdControl *sd)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[SD_CMD_LEN];

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return 0;
    }

    /* 使用 SEND_NUM_WR_BLOCKS 命令获取成功写入的块数 */
    /* SEND_NUM_WR_BLOCKS 是应用特有命令，需要先发送一个 APP_CMD 命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_APP_CMD;
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (sd_error(sd) != 0)
    {
        sd_disable(sd);
        sd_send(sd, 0xFFU);
        return 0;
    }

    /* 发送 SEND_NUM_WR_BLOCKS 命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_SEND_NUM_WR_BLOCKS;
    buf[5] = 0x01U;

    sd_cmd_r1(sd, buf);
    if (sd_error(sd) != 0)
    {
        sd_disable(sd);
        sd_send(sd, 0xFFU);
        return 0;
    }

    /* 接收 SD 卡返回的入块数 */
    status = sd_recv_data(sd, buf, 4);
    sd_disable(sd);
    sd_send(sd, 0xFFU);

    return (status == HAL_OK) ?
        (((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
            | ((uint32_t)buf[2] << 8) | buf[3])
        : 0;
}

uint32_t sd_read_reg(SdControl *sd, const uint8_t *cmd, uint8_t *data, size_t len)
{
    HAL_StatusTypeDef status = HAL_OK;

    sd_enable(sd);

    if ((status = sd_wait_busy(sd)) != HAL_OK)
    {
        sd_disable(sd);
        sd->status = (0x80000000U | status);
        return sd_error(sd);
    }

    /* 发送读 CSD 寄存器命令 */
    sd_cmd_r1(sd, cmd);
    if (sd_error(sd) == 0)
    {
        /* 接收寄存器数据 */
        sd_recv_data(sd, data, len);
    }

    sd_disable(sd);
    sd_send(sd, 0xFFU);
    return sd_error(sd);
}

size_t sd_block_num(SdControl *sd)
{
    uint8_t buf[16];
    size_t num = 0;

    /* 读 CSD 寄存器命令 */
    memset(buf, 0, SD_CMD_LEN);
    buf[0] = SD_SEND_CSD;
    buf[5] = 0x01U;

    sd_read_reg(sd, buf, buf, 16);
    if (sd_error(sd) == 0)
    {
        /* 根据 CSD 寄存器的内容计算 SD 卡容量 */
        if (buf[0] & 0x40U)
        {
            /* CSD V2 */
            uint32_t c_size = (((uint32_t)buf[8] << 8) | buf[9]);
            num = ((c_size + 1) << 10);
        }
        else
        {
            /* CSD V1 */
            uint32_t read_bl_len = (buf[5] & 0x0FU);
            uint32_t c_size = ((((uint32_t)buf[6] & 0x03U) << 10)
                | ((uint32_t)buf[7] << 2)
                | (buf[8] >> 6));
            uint32_t c_size_mult = (((buf[9] & 0x03U) << 1)
                | (((uint32_t)buf[10] & 0x80U) >> 7));

            /* BLOCKNR = (C_SIZE + 1) * 2 ^ (C_SIZE_MULT + 2)
            BLOCK_LEN = 2 ^ READ_BL_LEN
            num = BLOCKNR * BLOCK_LEN / SD_BLOCK_LEN */
            num = ((c_size + 1) << (read_bl_len + c_size_mult + 2 - 9));
        }
    }

    return num;
}

uint32_t sd_sync(SdControl *sd)
{
    sd->status = sd_wait_busy(sd);
    if (sd->status != HAL_OK)
    {
        sd->status |= 0x80000000U;
    }
    return sd_error(sd);
}
