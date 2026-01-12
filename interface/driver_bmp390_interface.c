/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_bmp390_interface_template.c
 * @brief     driver bmp390 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-05-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/05/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_bmp390_interface.h"

static bmp390_iic_config_t g_bmp390_iic_cfg;
static bmp390_spi_config_t g_bmp390_spi_cfg;


void bmp390_interface_iic_set_config(const bmp390_iic_config_t cfg)
{
    g_bmp390_iic_cfg = cfg;
}

void bmp390_interface_spi_set_config(const bmp390_spi_config_t cfg)
{
    g_bmp390_spi_cfg = cfg;
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t bmp390_interface_iic_init(void)
{
    (void)i2c_init(g_bmp390_iic_cfg.port, 400000);
    gpio_set_function(g_bmp390_iic_cfg.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(g_bmp390_iic_cfg.scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(g_bmp390_iic_cfg.scl_pin);
    gpio_pull_up(g_bmp390_iic_cfg.sda_pin);

    gpio_init(g_bmp390_iic_cfg.cs_pin);
    gpio_set_dir(g_bmp390_iic_cfg.cs_pin, GPIO_OUT);
    gpio_put(g_bmp390_iic_cfg.cs_pin, 1);

    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t bmp390_interface_iic_deinit(void)
{   
    i2c_deinit(g_bmp390_iic_cfg.port);
    gpio_set_function(g_bmp390_iic_cfg.scl_pin, GPIO_FUNC_SIO);
    gpio_set_function(g_bmp390_iic_cfg.sda_pin, GPIO_FUNC_SIO);
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int resp;
    resp = i2c_write_blocking(g_bmp390_iic_cfg.port, addr, &reg, sizeof(reg), true);

    if (resp == PICO_ERROR_GENERIC)
        return 1;

    resp = i2c_read_blocking(g_bmp390_iic_cfg.port, addr, buf, len, false);

    if (resp == PICO_ERROR_GENERIC)
        return 1;

    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp390_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t buffer_len = len + 1;
    uint8_t buffer[buffer_len];
    buffer[0] = reg;
    for (uint8_t index = 0; index < len; index++)
    {
        buffer[index + 1] = buf[index];
    }

    int resp;
    resp = i2c_write_blocking(g_bmp390_iic_cfg.port, addr, buf, buffer_len, true);

    if (resp == PICO_ERROR_GENERIC)
        return 1;

    return 0;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t bmp390_interface_spi_init(void)
{
    (void)spi_init(g_bmp390_spi_cfg.port, g_bmp390_spi_cfg.baudrate);
    gpio_set_function(g_bmp390_spi_cfg.scl_pin, GPIO_FUNC_SPI);
    gpio_set_function(g_bmp390_spi_cfg.sdo_pin, GPIO_FUNC_SPI);
    gpio_set_function(g_bmp390_spi_cfg.sdi_pin, GPIO_FUNC_SPI);

    gpio_init(g_bmp390_spi_cfg.cs_pin);
    gpio_set_dir(g_bmp390_spi_cfg.cs_pin, GPIO_OUT);
    gpio_put(g_bmp390_spi_cfg.cs_pin, 0);

    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t bmp390_interface_spi_deinit(void)
{   
    spi_deinit(g_bmp390_spi_cfg.port);
    gpio_set_function(g_bmp390_spi_cfg.scl_pin, GPIO_FUNC_SIO);
    gpio_set_function(g_bmp390_spi_cfg.sdi_pin, GPIO_FUNC_SIO);
    gpio_set_function(g_bmp390_spi_cfg.sdo_pin, GPIO_FUNC_SIO);
    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    // The MSB is set as 1 indicating a read command
    uint8_t command = 0x80 | reg;
    gpio_put(g_bmp390_spi_cfg.cs_pin, 0);

    int written = spi_write_blocking(g_bmp390_spi_cfg.port, &command, sizeof(command));
    if (written != 1)
    {
        gpio_put(g_bmp390_spi_cfg.cs_pin, 1);
        return 1;
    }

    int read = spi_read_blocking(g_bmp390_spi_cfg.port, 0x00, buf, len);
    if (read != len)
    {
        gpio_put(g_bmp390_spi_cfg.cs_pin, 1);
        return 1;
    }

    gpio_put(g_bmp390_spi_cfg.cs_pin, 1);

    return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t bmp390_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    // The MSB is set as 0 indicating a write command
    uint8_t command = 0x7F | reg;
    gpio_put(g_bmp390_spi_cfg.cs_pin, 0);

    int written = spi_write_blocking(g_bmp390_spi_cfg.port, &command, sizeof(command));
    if (written != 1)
    {
        gpio_put(g_bmp390_spi_cfg.cs_pin, 1);
        return 1;
    }

    written = spi_write_blocking(g_bmp390_spi_cfg.port, buf, len);
    if (written != len)
    {
        gpio_put(g_bmp390_spi_cfg.cs_pin, 1);
        return 1;
    }

    gpio_put(g_bmp390_spi_cfg.cs_pin, 1);

    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void bmp390_interface_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void bmp390_interface_debug_print(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    stdio_vprintf(fmt, args);
    va_end(args);
}

/**
 * @brief     interface receive callback
 * @param[in] type interrupt type
 * @note      none
 */
void bmp390_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case BMP390_INTERRUPT_STATUS_FIFO_WATERMARK :
        {
            bmp390_interface_debug_print("bmp390: irq fifo watermark.\n");
            
            break;
        }
        case BMP390_INTERRUPT_STATUS_FIFO_FULL :
        {
            bmp390_interface_debug_print("bmp390: irq fifo full.\n");
            
            break;
        }
        case BMP390_INTERRUPT_STATUS_DATA_READY :
        {
            bmp390_interface_debug_print("bmp390: irq data ready.\n");
            
            break;
        }
        default :
        {
            bmp390_interface_debug_print("bmp390: unknown code.\n");
            
            break;
        }
    }
}
