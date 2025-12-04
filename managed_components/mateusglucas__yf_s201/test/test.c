#include "unity.h"
#include "yf_s201.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static size_t init_deinit_sensor_memleak(){
    size_t free_heap_start = heap_caps_get_free_size(MALLOC_CAP_8BIT);

    yf_s201_handle_t handle = yf_s201_init(GPIO_NUM_10);
    TEST_ASSERT(handle!=NULL);

    size_t free_heap_middle = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    TEST_ASSERT(free_heap_middle<free_heap_start);

    esp_err_t err = yf_s201_deinit(handle);
    TEST_ASSERT(err==ESP_OK);

    size_t free_heap_end = heap_caps_get_free_size(MALLOC_CAP_8BIT);

    return free_heap_start-free_heap_end;
}

TEST_CASE("init/deinit memory leak", "[yf-s201]"){
    size_t mem_leak_bytes = init_deinit_sensor_memleak();

    /* don't account first init/deinit because it allocates gpio 
       isr service, resulting in an apparent leak */
    mem_leak_bytes = 0;
    
    const int n_loop = 10;
    for(int i=0; i<n_loop; i++){
        mem_leak_bytes += init_deinit_sensor_memleak();
    }

    TEST_ASSERT(mem_leak_bytes==0);
}

TEST_CASE("liters increment and reset", "[yf-s201]"){
    const gpio_num_t sensor_pin = GPIO_NUM_19;

    yf_s201_handle_t handle = yf_s201_init(sensor_pin);
    TEST_ASSERT(handle!=NULL);

    /* modify mode to input/output to be able to force pin levels */
    TEST_ASSERT(gpio_set_direction(sensor_pin, GPIO_MODE_INPUT_OUTPUT) == ESP_OK);
    TEST_ASSERT(gpio_set_level(sensor_pin, 0) == ESP_OK);

    vTaskDelay(1);

    yf_s201_reset_liters(handle);

    vTaskDelay(1);

    TEST_ASSERT(yf_s201_get_liters(handle) == 0);

    const int n_pulses = 10;
    for(int i=0; i<n_pulses; i++){
        TEST_ASSERT(gpio_set_level(sensor_pin, 1)==ESP_OK);
        vTaskDelay(1);

        TEST_ASSERT(gpio_set_level(sensor_pin, 0)==ESP_OK);
        vTaskDelay(1);
        
        TEST_ASSERT(yf_s201_get_liters(handle) == (i+1)*((double) 1./(7.5*60)));
    }

    yf_s201_reset_liters(handle);

    vTaskDelay(1);

    TEST_ASSERT(yf_s201_get_liters(handle) == 0);

    TEST_ASSERT(yf_s201_deinit(handle) == ESP_OK);
}