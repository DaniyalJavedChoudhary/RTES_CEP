#include "yf_s201.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define LITERS_PER_COUNT ((double) 1./(7.5*60))
// YF-S201
// Freq[Hz] = 7.5 * flux[L/min]

#define GOTO_ON_FALSE(a, goto_tag, log_tag, format, ...) do {                     \
        if (unlikely(!(a))) {                                                                   \
            ESP_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__, ##__VA_ARGS__);        \
            goto goto_tag;                                                                      \
        }                                                                                       \
    } while (0)

const char* TAG = "yf-s201";

struct yf_s201_t{
    gpio_num_t gpio_num;
    TaskHandle_t task;
    SemaphoreHandle_t cnt_mutex;
    uint64_t cnt;
};

static esp_err_t yf_s201_config_gpio(struct yf_s201_t* device);
static void yf_s201_task(void *arg);
static IRAM_ATTR void yf_s201_isr_handler(void *arg);
static uint64_t yf_s201_safe_read_count(struct yf_s201_t* device);
static void yf_s201_safe_write_count(struct yf_s201_t* device, uint64_t cnt);

yf_s201_handle_t yf_s201_init(gpio_num_t gpio_num){
    struct yf_s201_t* device = malloc(sizeof(struct yf_s201_t));
    GOTO_ON_FALSE(device != NULL, err0, TAG, "fail to alloc device handle");

    device->cnt=0;
    device->gpio_num = gpio_num;

    device->cnt_mutex = xSemaphoreCreateMutex();
    GOTO_ON_FALSE(device->cnt_mutex != NULL, err1, TAG, "fail to create mutex");

    BaseType_t ret = xTaskCreate(yf_s201_task, "YF-S201 task", 4*1024, device, 1, &device->task);
    GOTO_ON_FALSE(ret == pdPASS, err2, TAG, "fail to create task");

    esp_err_t err = yf_s201_config_gpio(device);
    GOTO_ON_FALSE(err == ESP_OK, err3, TAG, "fail to config gpio");

    return device;

err3:
    vTaskDelete(device->task);
err2:
    vSemaphoreDelete(device->cnt_mutex);
err1:
    free(device);
err0:
    return NULL;
}

void yf_s201_task(void *arg){
    struct yf_s201_t* device = (struct yf_s201_t*) arg;

    for(;;){
        uint64_t cnt = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(device->cnt_mutex, portMAX_DELAY);
        device->cnt += cnt;
        xSemaphoreGive(device->cnt_mutex);
    }
    vTaskDelete(NULL);
}

esp_err_t yf_s201_config_gpio(struct yf_s201_t* device){
    gpio_config_t conf ={
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL<<device->gpio_num,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    
    esp_err_t ret = gpio_config(&conf);
    GOTO_ON_FALSE(ret == ESP_OK, err0, TAG, "fail to config gpio");

    ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
    bool isr_already_installed = (ret == ESP_ERR_INVALID_STATE);
    GOTO_ON_FALSE(ret == ESP_OK || ret == ESP_ERR_INVALID_STATE, err0, TAG, "fail to install gpio isr service");

    ret = gpio_isr_handler_add(device->gpio_num, yf_s201_isr_handler, device);
    GOTO_ON_FALSE(ret == ESP_OK, err1, TAG, "fail to add gpio isr handler");

    return ESP_OK;

err1:
    if(!isr_already_installed){
        gpio_uninstall_isr_service();
    }
err0:
    return ret;
}

void yf_s201_isr_handler(void *arg){
    struct yf_s201_t* device = (struct yf_s201_t*) arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(device->task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

double yf_s201_get_liters(yf_s201_handle_t handle){
    return LITERS_PER_COUNT*yf_s201_safe_read_count(handle);
}

void yf_s201_reset_liters(yf_s201_handle_t handle){
    yf_s201_safe_write_count(handle, 0);
}

uint64_t yf_s201_safe_read_count(struct yf_s201_t* device){
    xSemaphoreTake(device->cnt_mutex, portMAX_DELAY);
    uint64_t cnt = device->cnt;
    xSemaphoreGive(device->cnt_mutex);

    return cnt;
}

void yf_s201_safe_write_count(struct yf_s201_t* device, uint64_t cnt){
    xSemaphoreTake(device->cnt_mutex, portMAX_DELAY);
    device->cnt = cnt;
    xSemaphoreGive(device->cnt_mutex);
}

esp_err_t yf_s201_deinit(yf_s201_handle_t handle){
    esp_err_t ret = gpio_isr_handler_remove(handle->gpio_num);
    GOTO_ON_FALSE(ret == ESP_OK, err, TAG, "fail to remove gpio isr handler");

    gpio_config_t conf ={
        .mode = GPIO_MODE_DISABLE,
        .pin_bit_mask = 1ULL<<handle->gpio_num,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    ret = gpio_config(&conf);
    GOTO_ON_FALSE(ret == ESP_OK, err, TAG, "fail to disable gpio");

    vTaskDelete(handle->task);
    vSemaphoreDelete(handle->cnt_mutex);
    free(handle);

    return ESP_OK;

err:
    return ret;
}