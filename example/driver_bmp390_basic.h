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
 * @file      driver_bmp390_basic.h
 * @brief     driver bmp390 basic header file
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

#ifndef DRIVER_BMP390_BASIC_H
#define DRIVER_BMP390_BASIC_H

#include "driver_bmp390_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup bmp390_example_driver bmp390 example driver function
 * @brief    bmp390 example driver modules
 * @ingroup  bmp390_driver
 * @{
 */

typedef struct {
    bmp390_interface_t           bus_type;
    bmp390_address_t             i2c_device_addr;
    bmp390_spi_wire_t            spi_wire_type;
    bmp390_bool_t                enable_i2c_watchdog_timer;
    bmp390_iic_watchdog_period_t i2c_watch_dog_period;
    bmp390_bool_t                enable_pressure_measurements;
    bmp390_bool_t                enable_temperature_measurements;
    bmp390_oversampling_t        pressure_oversampling;
    bmp390_oversampling_t        temperature_oversampling;
    bmp390_odr_t                 output_data_rate;
    bmp390_filter_coefficient_t  filter_coefficient;   
} bmp390_device_config_t;

/**
 * @brief Initialize bmp390 config to its default values
 * @param[out] config Where the default config values will be stored
 */
void bmp390_device_config_set_defaults(bmp390_device_config_t *config);

/**
 * @brief     basic example init
 * @param[in] interface chip interface
 * @param[in] addr_pin iic device address
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t bmp390_basic_init(bmp390_device_config_t config);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t bmp390_basic_deinit(void);

/**
 * @brief      basic example read
 * @param[out] *temperature_c pointer a converted temperature data buffer
 * @param[out] *pressure_pa pointer a converted pressure data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bmp390_basic_read(float *temperature_c, float *pressure_pa);

uint8_t bmp390_basic_read_status(uint8_t *status);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
