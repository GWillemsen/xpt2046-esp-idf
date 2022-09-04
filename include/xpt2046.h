/**
 * @file xpt2046.h
 * @author Giel Willemsen
 * @brief The API definition for the XPT2046 device driver.
 * @version 0.1 Initial version
 * @date 2022-09-04
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

#pragma once
#ifndef XPT2046_H_
#define XPT2046_H_
#include <stdint.h>
#include <driver/spi_master.h>
#include <esp_err.h>

// Forward declares
typedef struct xpt2046 xpt2046_t; // Forward declare so we can keep the implementation private.

// Helper typedefs
typedef struct xpt2046 *xpt2046_handle_t;

// Structs
typedef struct xpt2046_coord {
    int x;  ///< The x of a coordinate
    int y;  ///< The y of a coordinate
} xpt2046_coord_t;

typedef struct xpt2046_spi_config {
    spi_host_device_t host; ///< Handle to a SPI peripheral used to create the SPI device on.
    int clock_speed_hz;     ///< The frequency of the SPI bus in Hz.
    int cs_io_num;          ///< The pin number of the GPIO to use for the CS line.
} xpt2046_spi_config_t;

typedef struct xpt2046_config {
    xpt2046_spi_config_t spi_config;    ///< The SPI bus configuration
    xpt2046_coord_t screen_size;        ///< The actual coordinate size of the screen.
    uint8_t oversample_count;           ///< The number of samples to take for a single measurement.
    uint8_t moving_average_count;       ///< The number of measurements to average to calculate the current position.
} xpt2046_config_t;

// Public functions

/**
 * @brief Creates a new XPT2046 from the config and returns the handle to it.
 * 
 * @param handle The handle that the XPT2046 is created at.
 * @param config The configuration of the new XPT2046 device.
 * @return esp_err_t The success code for creating the new instance.
 */
extern esp_err_t xpt2046_initialize(xpt2046_handle_t *handle, const xpt2046_config_t *config);

/**
 * @brief Deallocates all resources used by the given handle (including the handle itself!).
 * 
 * @param handle The handle to free.
 * @return esp_err_t The success code for freeing all the allocated resources.
 */
extern esp_err_t xpt2046_free(xpt2046_handle_t handle);

/**
 * @brief Make a new measurement and return the result of this measurement.
 * 
 * @param handle The instance to make a new measurement with.
 * @param position The resulting position on the screen.
 * @param pressure The pressure that the screen was touched with.
 * @return esp_err_t The success code for make a new measurement.
 */
extern esp_err_t xpt2046_update(xpt2046_handle_t handle, xpt2046_coord_t *position, int16_t *pressure);

#endif // XPT2046_H_
