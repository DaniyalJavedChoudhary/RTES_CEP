#include <stdio.h>
#include "esp_log.h"
#include "yf_s201.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FLOW_METER_GPIO GPIO_NUM_20

void app_main(void)
{
    yf_s201_handle_t handle = yf_s201_init(FLOW_METER_GPIO);
    ESP_ERROR_CHECK(handle!=NULL ? ESP_OK : ESP_FAIL);

    for(;;){
        double cnt = yf_s201_get_liters(handle);
        ESP_LOGI("main", "Vol: %f L", cnt);
        vTaskDelay(10);
    }
}