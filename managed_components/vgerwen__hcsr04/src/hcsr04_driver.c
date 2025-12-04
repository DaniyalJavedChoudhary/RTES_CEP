/**
 * @file hcsr04-driver.c
 * @author Aad van Gerwen
 * @brief hcsr04 driver
 * @version 0.1
 * @date 2025-03-18
 **/


#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "hcsr04_driver.h"

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10   // HC SR04 start pulse with 10 usec*/
#define PING_TIMEOUT 6000       // echo input high max 6 msec after trigger puls*/
#define ROUNDTRIP_CM 58

// GPIO pins to HC SR04 module
#define ESP_HCSR04_TRIGGER_PIN    CONFIG_TRIGGER_PIN   // define trigger IO pin */
#define ESP_HCSR04_ECHO_PIN       CONFIG_ECHO_PIN      // define echo IO pin */


/* Logging tag */
static const char *log_tag = "HCRS04 tag"; /*  Ultrasonic logging tag */
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* 
    Private functions
*/

static esp_err_t ultrasonic_measure_raw(uint32_t max_time_us, uint32_t *time_us)
{
    esp_err_t return_value = ESP_OK;
    int64_t   echo_start;
    int64_t   time;

    if (time_us != NULL) {
        portENTER_CRITICAL(&mux);

        // Ping: Low for 4 us, then high 10 us
        gpio_set_level(ESP_HCSR04_TRIGGER_PIN, 0);
        esp_rom_delay_us(TRIGGER_LOW_DELAY);
        gpio_set_level(ESP_HCSR04_TRIGGER_PIN, 1);
        esp_rom_delay_us(TRIGGER_HIGH_DELAY);
        gpio_set_level(ESP_HCSR04_TRIGGER_PIN, 0);

        // Previous ping isn't ended
        if (gpio_get_level(ESP_HCSR04_ECHO_PIN))
            return_value = ESP_ERR_ULTRASONIC_PING;

        // Wait for echo
        echo_start = esp_timer_get_time();
        while (!gpio_get_level(ESP_HCSR04_ECHO_PIN) && (return_value == ESP_OK)) {
            time = esp_timer_get_time();
            if ( time - echo_start >= PING_TIMEOUT)
                return_value = ESP_ERR_ULTRASONIC_PING;
        }

        // Echo deteced wait for end of echo
        echo_start = esp_timer_get_time();
        time = echo_start;
        while ((gpio_get_level(ESP_HCSR04_ECHO_PIN)) && (return_value == ESP_OK)) {
            time = esp_timer_get_time();
            if (time - echo_start >= max_time_us) { 
                return_value = ESP_ERR_ULTRASONIC_ECHO_TIMEOUT;
            }
        }    
        portEXIT_CRITICAL(&mux);
        *time_us = time - echo_start;
    }
    else {
        return_value = ESP_ERR_INVALID_ARG;  // Invalid argument
    }
    return return_value;
}

/*
   API Interface functions
*/
esp_err_t UltrasonicInit(void)
{
    gpio_set_direction(ESP_HCSR04_TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction( ESP_HCSR04_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(ESP_HCSR04_TRIGGER_PIN, 0);

    return ESP_OK;
}


esp_err_t UltrasonicMeasure(uint32_t max_distance, uint32_t *distance)
{
    uint32_t    time_us;
    esp_err_t   return_value = ESP_OK;

    if (distance != NULL) {
        return_value = ultrasonic_measure_raw(max_distance * ROUNDTRIP_CM, &time_us);
        *distance = time_us / ROUNDTRIP_CM;
    }
    else {
        return_value = ESP_ERR_INVALID_ARG;
    }
    return return_value;
}


void UltrasonicAssert(esp_err_t error_code)
{
    if (error_code != ESP_OK) {
        ESP_LOGI(log_tag, "Measurement error: %x\n", error_code);
    }
}