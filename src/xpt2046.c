/**
 * @file xpt2046.c
 * @author Giel Willemsen
 * @brief The API implementation for the XPT2046 device driver.
 * @version 0.1 2022-09-04 Initial version
 * @version 0.2 2022-09-06 Add option for mirroring and flipping of XY values.
 * @date 2022-09-06
 * 
 * @copyright
 * MIT License
 * 
 * Copyright (c) 2022 Giel Willemsen
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
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <esp_err.h>
#include <esp_log.h>

#include "xpt2046.h"

// Macros
#define CMD_VAL_Z1          0b10110000 //  0xB1
#define CMD_VAL_Z2          0b11000000 //  0xC1
#define CMD_VAL_X           0b10010000 //  0x91
#define CMD_VAL_Y           0b11010000 //  0xD1
#define CMD_BIT_LENGTH      8
#define CMD_READ_BIT_LENGTH 16
#define ADC_MAX_VALUE       4096

// Private structs
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z1;
    uint16_t z2;
} raw_data_t;

// Private functions prototypes
static esp_err_t read_adc_data(xpt2046_handle_t handle, raw_data_t *data);
static esp_err_t read_register(xpt2046_handle_t handle, uint8_t reg, int16_t *result);
static void add_to_avg(xpt2046_handle_t handle, raw_data_t data);
static xpt2046_coord_t calculate_avg(xpt2046_handle_t handle);
static xpt2046_coord_t map_adc_values(xpt2046_handle_t handle);

// Private 'public' structs
typedef struct xpt2046
{
    spi_device_handle_t spi;
    xpt2046_screen_config_t screen_config;
    uint8_t oversample_count;
    uint8_t moving_average_count;
    int16_t *avg_x_value;
    int16_t *avg_y_value;
    uint8_t avg_xy_values_in_use;
} xpt2046_t;

// Public functions

extern esp_err_t xpt2046_initialize(xpt2046_handle_t *new_handle, const xpt2046_config_t *config)
{
    if (new_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    xpt2046_handle_t handle = calloc(1, sizeof(xpt2046_t));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }
    *new_handle = handle;

    handle->avg_x_value = calloc(1, CONFIG_XPT2046_AVG_COUNT * sizeof(handle->avg_x_value[0]));
    if (handle->avg_x_value == NULL) {
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    handle->avg_y_value = calloc(1, CONFIG_XPT2046_AVG_COUNT * sizeof(handle->avg_y_value[0]));
    if (handle->avg_y_value == NULL) {
        free(handle->avg_x_value);
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    handle->avg_xy_values_in_use = 0;
    handle->screen_config = config->screen_config;
    handle->oversample_count = config->oversample_count;
    handle->moving_average_count = config->moving_average_count;
        
    spi_device_interface_config_t dev_config = {
        .command_bits = CMD_BIT_LENGTH,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = config->spi_config.clock_speed_hz,
        .spics_io_num = config->spi_config.cs_io_num,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
        .queue_size = 3
    };
    esp_err_t err = spi_bus_add_device(config->spi_config.host, &dev_config, &handle->spi);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK) {
        free(handle->avg_x_value);
        free(handle->avg_y_value);
        free(handle);
        return err;
    }
    return ESP_OK;
}

extern esp_err_t xpt2046_free(xpt2046_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = spi_bus_remove_device(handle->spi);
    if (err != ESP_OK) {
        return err;
    }

    free(handle->avg_x_value);
    free(handle->avg_y_value);
    free(handle);
    return ESP_OK;
}

extern esp_err_t xpt2046_update(xpt2046_handle_t handle, xpt2046_coord_t *position, int16_t *pressure)
{
    raw_data_t data = { 0 };
    esp_err_t err = read_adc_data(handle, &data);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK) {
        return err;
    }

    add_to_avg(handle, data);

    *position = map_adc_values(handle);

    int16_t z = (data.z1 + 4096) - data.z2;
    *pressure = z;
    return ESP_OK;
}

static esp_err_t read_adc_data(xpt2046_handle_t handle, raw_data_t *data)
{
    int32_t x = 0;
    int32_t y = 0;
    int16_t z1 = 0;
    int16_t z2 = 0;
    int16_t x_dummy = 0;
    
    esp_err_t err = read_register(handle, CMD_VAL_Z1, &z1);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK) {
        return err;
    }

    err = read_register(handle, CMD_VAL_Z2, &z2);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK) {
        return err;
    }

    err = read_register(handle, CMD_VAL_X, &x_dummy);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < handle->oversample_count; i++) {

        int16_t x1 = 0;
        int16_t y1 = 0;
        err = read_register(handle, CMD_VAL_X, &x1);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        if (err != ESP_OK) {
            return err;
        }

        err = read_register(handle, CMD_VAL_Y, &y1);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        if (err != ESP_OK) {
            return err;
        }
        x += (x1 >> 3);
        y += (y1 >> 3);
    }

    data->z1 = z1 >> 3;
    data->z2 = z2 >> 3;
    data->x = (x / CONFIG_XPT2046_OVER_SAMPLE);
    data->y = (y / CONFIG_XPT2046_OVER_SAMPLE);
    return ESP_OK;
}

static esp_err_t read_register(xpt2046_handle_t handle, uint8_t reg, int16_t *result)
{
    uint8_t *data = heap_caps_malloc(sizeof(uint8_t) * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (data == NULL) {
        return ESP_ERR_NO_MEM;
    }
    spi_transaction_t t = {
        .length = CMD_READ_BIT_LENGTH + CMD_BIT_LENGTH,
	    .rxlength = CMD_READ_BIT_LENGTH,
	    .cmd = reg,
	    .rx_buffer = data,
	    .flags = 0
    };

    esp_err_t err = spi_device_transmit(handle->spi, &t);
    if (err == ESP_OK) {
        *result = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    }
    free(data);
    return err;
}

static void add_to_avg(xpt2046_handle_t handle, raw_data_t coordinates)
{
    if (handle->avg_xy_values_in_use >= handle->moving_average_count) {
        memmove(handle->avg_x_value, handle->avg_x_value + 1, (handle->moving_average_count - 1) * 2);
        memmove(handle->avg_y_value, handle->avg_y_value + 1, (handle->moving_average_count - 1) * 2);
        handle->avg_x_value[handle->moving_average_count - 1] = coordinates.x;
        handle->avg_y_value[handle->moving_average_count - 1] = coordinates.y;
    } else {
        handle->avg_x_value[handle->avg_xy_values_in_use] = coordinates.x;
        handle->avg_y_value[handle->avg_xy_values_in_use] = coordinates.y;
        handle->avg_xy_values_in_use++;
    }
}

static xpt2046_coord_t calculate_avg(xpt2046_handle_t handle)
{
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for (size_t i = 0; i < handle->avg_xy_values_in_use; i++) {
        x_sum += handle->avg_x_value[i];
        y_sum += handle->avg_y_value[i];
    }
    xpt2046_coord_t result = { 0  };
    if (handle->avg_xy_values_in_use != 0) {
        result.x = x_sum / handle->avg_xy_values_in_use;
        result.y = y_sum / handle->avg_xy_values_in_use;
    }
    return result;
}

static xpt2046_coord_t map_adc_values(xpt2046_handle_t handle)
{
    xpt2046_coord_t position = calculate_avg(handle);
    if (handle->screen_config.flip_xy) {
        int16_t temp = position.x;
        position.x = position.y;
        position.y = temp;
    }
    if (handle->screen_config.mirror_x) {
        position.x = ADC_MAX_VALUE - position.x;
    }
    if (handle->screen_config.mirror_y) {
        position.y = ADC_MAX_VALUE - position.y;
    }
    position.x = (position.x / ((double)ADC_MAX_VALUE)) * handle->screen_config.size.x;
    position.y = (position.y / ((double)ADC_MAX_VALUE)) * handle->screen_config.size.y;
    return position;
}
