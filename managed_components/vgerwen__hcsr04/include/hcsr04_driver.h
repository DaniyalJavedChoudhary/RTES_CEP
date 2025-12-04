/**
 * @file hcsr04-driver.h
 * @author Aad van Gerwen
 * @brief ESP IDF hcsr04 driver
 * @version 0.1
 * @date 2025-03-18
 **/

#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202


/**
 * @brief Init ranging module
 *
 * @param dev Pointer to the device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t UltrasonicInit(void);


/**
 * @brief Measure distance in centimeters
 *
 * @param [in] max_distance Maximal distance to measure, centimeters
 * @param[out] distance Distance in centimeters
 * @return `ESP_OK` on success, otherwise:
 *         - ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
 *         - ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
 *         - ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
 */
esp_err_t UltrasonicMeasure(uint32_t max_distance, uint32_t *distance);


/**
 * @brief If error_code != ESP_OK log errorcode
 *
 * @param [in] error_code erroro code to be printed

 * */
void UltrasonicAssert(esp_err_t error_code);


#ifdef __cplusplus
}
#endif
