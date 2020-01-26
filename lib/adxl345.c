/**
 * ADXL345 加速度传感器驱动.
 */

#include "stm32f1xx_hal.h"

#include "util.h"
#include "adxl345.h"


HAL_StatusTypeDef adxl345_i2c_init(
    Adxl345I2cControl *adxl345,
    I2C_HandleTypeDef *i2c,
    uint8_t addr,
    uint32_t timeout
)
{
    adxl345->mode = ADXL345_I2C;
    /* 设置对应的读写函数指针 */
    adxl345->write = (HAL_StatusTypeDef (*)(void *, uint8_t, uint8_t))adxl345_i2c_write;
    adxl345->read = (HAL_StatusTypeDef (*)(void *, uint8_t, uint8_t *))adxl345_i2c_read;
    adxl345->readmb = (HAL_StatusTypeDef (*)(void *, uint8_t, uint16_t *))adxl345_i2c_readmb;
    adxl345->i2c = i2c;
    adxl345->addr = addr;
    adxl345->timeout = timeout;

    return HAL_OK;
}

HAL_StatusTypeDef adxl345_i2c_write(
    Adxl345I2cControl *adxl345,
    uint8_t addr,
    uint8_t data
)
{
    return HAL_I2C_Mem_Write(
        adxl345->i2c,
        adxl345->addr,
        addr,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        adxl345->timeout
    );
}

HAL_StatusTypeDef adxl345_i2c_read(
    Adxl345I2cControl *adxl345,
    uint8_t addr,
    uint8_t *data
)
{
    return HAL_I2C_Mem_Read(
        adxl345->i2c,
        adxl345->addr,
        addr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        adxl345->timeout
    );
}

HAL_StatusTypeDef adxl345_i2c_readmb(
    Adxl345I2cControl *adxl345,
    uint8_t addr,
    uint16_t *data
)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[2];

    status = HAL_I2C_Mem_Read(
        adxl345->i2c,
        adxl345->addr,
        addr | ADXL345_MB,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        2,
        adxl345->timeout
    );

    if (HAL_OK == status)
    {
        *data = (((uint16_t)buf[1] << 8) | buf[0]);
    }

    return status;
}

HAL_StatusTypeDef adxl345_spi_init(
    Adxl345SpiControl *adxl345,
    SPI_HandleTypeDef *spi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint32_t timeout
)
{
    adxl345->mode = ADXL345_SPI;
    /* 设置相应读写函数指针 */
    adxl345->write = (HAL_StatusTypeDef (*)(void *, uint8_t, uint8_t))adxl345_spi_write;
    adxl345->read = (HAL_StatusTypeDef (*)(void *, uint8_t, uint8_t *))adxl345_spi_read;
    adxl345->readmb = (HAL_StatusTypeDef (*)(void *, uint8_t, uint16_t *))adxl345_spi_readmb;
    adxl345->cs_port = cs_port;
    adxl345->cs_pin = cs_pin;
    adxl345->timeout = timeout;

    return HAL_OK;
}

HAL_StatusTypeDef adxl345_spi_write(
    Adxl345SpiControl *adxl345,
    uint8_t addr,
    uint8_t data
)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[2] = {addr, data};

    delayus(10);
    /* SPI 片选使能 */
    HAL_GPIO_WritePin(adxl345->cs_port, adxl345->cs_pin, GPIO_PIN_RESET);
    delayus(10);
    status = HAL_SPI_Transmit(adxl345->spi, buf, 2, adxl345->timeout);
    delayus(10);
    /* SPI 取消片选 */
    HAL_GPIO_WritePin(adxl345->cs_port, adxl345->cs_pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef adxl345_spi_read(
    Adxl345SpiControl *adxl345,
    uint8_t addr,
    uint8_t *data
)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* SPI 地址要置读标志位 */
    addr |= ADXL345_RW;

    delayus(10);
    /* SPI 片选使能 */
    HAL_GPIO_WritePin(adxl345->cs_port, adxl345->cs_pin, GPIO_PIN_RESET);
    delayus(10);
    status = HAL_SPI_Transmit(adxl345->spi, &addr, 1, adxl345->timeout);
    if (HAL_OK == status)
    {
        status = HAL_SPI_Receive(adxl345->spi, data, 1, adxl345->timeout);
    }
    /* 取消 SPI 片选 */
    HAL_GPIO_WritePin(adxl345->cs_port, adxl345->cs_pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef adxl345_spi_readmb(
    Adxl345SpiControl *adxl345,
    uint8_t addr,
    uint16_t *data
)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t buf[2];

    /* SPI 地址要置读标志位 */
    addr |= ADXL345_RW;

    delayus(10);
    /* SPI 片选使能 */
    HAL_GPIO_WritePin(adxl345->cs_port, adxl345->cs_pin, GPIO_PIN_RESET);
    delayus(10);
    status = HAL_SPI_Transmit(adxl345->spi, &addr, 1, adxl345->timeout);
    if (HAL_OK == status)
    {
        status = HAL_SPI_Receive(adxl345->spi, buf, 2, adxl345->timeout);
    }
    /* 取消 SPI 片选 */
    HAL_GPIO_WritePin(adxl345->cs_port, adxl345->cs_pin, GPIO_PIN_SET);

    if (HAL_OK == status)
    {
        *data = (((int16_t)buf[1] << 8) | buf[0]);
    }

    return status;
}

HAL_StatusTypeDef adxl345_init(Adxl345Control *adxl345)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t devid = 0;

    for (size_t i = 0; i < 3; ++i)
    {
        status = (*adxl345->read)(adxl345, ADXL345_DEVID, &devid);
        if ((HAL_OK == status) && (ADXL345_DEVICE_ID == devid))
        {
            break;
        }
    }

    if (status != HAL_OK)
    {
        return status;
    }
    if (devid != ADXL345_DEVICE_ID)
    {
        return HAL_ERROR;
    }

    /* 全分辨率，量程 +-16g */
    if ((status = (*adxl345->write)(adxl345, ADXL345_DATA_FORMAT, 0x0B)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_OFSX, 0)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_OFSY, 0)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_OFSZ, 0)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_POWER_CTL, 0x08)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_BW_RATE, 0x0E)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_INT_ENABLE, 0x00)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_INT_MAP, 0x00)) != HAL_OK)
    {
        return status;
    }
    if ((status = (*adxl345->write)(adxl345, ADXL345_FIFO_CTL, 0x00)) != HAL_OK)
    {
        return status;
    }

    return status;
}

HAL_StatusTypeDef adxl345_get_acc(
    Adxl345Control *adxl345,
    int32_t *x,
    int32_t *y,
    int32_t *z
)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint16_t data = 0;

    status = (*adxl345->readmb)(adxl345, ADXL345_DATAX0, &data);
    if (HAL_OK == status)
    {
        *x = (int16_t)data;
    }
    else
    {
        return status;
    }

    status = (*adxl345->readmb)(adxl345, ADXL345_DATAY0, &data);
    if (HAL_OK == status)
    {
        *y = (int16_t)data;
    }
    else
    {
        return status;
    }

    status = (*adxl345->readmb)(adxl345, ADXL345_DATAZ0, &data);
    if (HAL_OK == status)
    {
        *z = (int16_t)data;
    }
    else
    {
        return status;
    }

    return status;
}
