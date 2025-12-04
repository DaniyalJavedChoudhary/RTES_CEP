#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief Device handle
 */
typedef struct yf_s201_t* yf_s201_handle_t;

/**
 * @brief Initialize device
 * @param gpio_num gpio number
 * @return device handle
 */
yf_s201_handle_t yf_s201_init(gpio_num_t gpio_num);

/**
 * @brief Get volume in liters
 * @param handle device handle
 * @return volume in liters
 */
double yf_s201_get_liters(yf_s201_handle_t handle);

/**
 * @brief Reset liters counter
 * @param handle device handle
 */
void yf_s201_reset_liters(yf_s201_handle_t handle);

/**
 * @brief Deinitialize device
 * @param handle device handle
 * @return ESP_OK on success or ESP_ERR code on fail
 */
esp_err_t yf_s201_deinit(yf_s201_handle_t handle);