#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_netif.h"

#ifdef CONFIG_ESP_TASK_WDT_EN
#include "esp_task_wdt.h"
#endif

#if defined(__has_include)
#  if __has_include("esp_vfs_spiffs.h")
#    include "esp_vfs_spiffs.h"
#  else
#    include "esp_spiffs.h"
#  endif
#else
#  include "esp_vfs_spiffs.h"
#endif

#include "cJSON.h"
#include "ultrasonic.h"

/* ===================================================================
 * APPLICATION CONSTANTS AND CONFIGURATION (MISRA C Compliant)
 * =================================================================== */

static const char *TAG = "WEB";

/* -------- Pump GPIO Configuration -------- */
#define PUMP1_GPIO              (18U)
#define PUMP2_GPIO              (19U)
#define PUMP_ACTIVE_LEVEL       (0U)
#define PUMP_OPEN_DRAIN         (1U)

/* -------- Flow Sensor Configuration -------- */
#define FLOW_GPIO               GPIO_NUM_17
/* YF-S201 calibration: Empirically determined from testing
   Datasheet: ~450 pulses/L (often incorrect for clones)
   Tested: 17000 pulses/L brings readings within 1-30 L/min range
   Adjust ±10% if readings seem off after real-world testing */
#define PULSES_PER_LITER        (17000.0f)
#define MEASURE_INTERVAL_MS     (2000U)

/* -------- Ultrasonic Sensor Configuration -------- */
#define US_TANK_HEIGHT_CM       (13.5f)
#define US_TANK_SURFACE_AREA    (188.0f)
#define US_MEASURE_MAX_CM       (400U)
#define US_MEASURE_INTERVAL_MS  (5000U)
/* 50% threshold: 50% of 2.54L = 1.225L, 50% of 13.5cm height = 7 cm distance */
#define US_HALF_DISTANCE_CM     (7.0f)
#define US_HALF_LITERS          (1.225f)

/* -------- HTTP Server Configuration -------- */
#define HTTP_STACK_SIZE         (4096U)
#define HTTP_MAX_HANDLERS       (32U)
#define HTTP_BUFFER_SIZE        (1024U)
#define SPIFFS_MAX_FILES        (10U)
#define SPIFFS_BASE_PATH        "/spiffs"

/* -------- Task Configuration -------- */
#define TASK_STACK_SIZE         (4096U)
#define STARTUP_TASK_STACK      (8192U)
#define TASK_PRIORITY           (5U)

/* ===================================================================
 * TYPE DEFINITIONS AND STRUCTURES
 * =================================================================== */

typedef struct {
    float last_lpm;
    float last_liters;
    uint32_t last_pulses;
} flow_state_t;

/* ===================================================================
 * GLOBAL STATE VARIABLES (MISRA: minimize global state)
 * =================================================================== */

/* Flow sensor state (ISR-safe) */
static atomic_uint_fast32_t s_pulse_count = ATOMIC_VAR_INIT(0U);

/* Flow state snapshot */
static flow_state_t s_flow_state = {0.0f, 0.0f, 0U};

/* Ultrasonic sensor state */
static uint32_t s_ultrasonic_distance_cm = 0U;
static float s_ultrasonic_litre = 0.0f;

/* Cumulative flow tracking */
static float s_cumulative_liters = 0.0f;

/* Pump state tracking (for reliable state queries in open-drain mode) */
static bool s_pump1_state = false;
static bool s_pump2_state = false;

/* Semaphore used to signal SPIFFS mount completion */
static SemaphoreHandle_t spiffs_mounted_sem = NULL;

/* ===================================================================
 * FORWARD DECLARATIONS (MISRA: All functions declared before use)
 * =================================================================== */

/* Flow sensor functions */
static void flow_pulse_isr(void *arg);  /* IRAM_ATTR applied at definition only */
static void flow_init(void);
void flow_get_latest(float *out_lpm, uint32_t *out_pulses, float *out_liters);
void flow_task(void *pvParameter);

/* Pump control functions */
static void init_pumps(void);
static esp_err_t pump_set_state(int pump_id, bool on);
static bool pump_get_state(int pump_id);

/* HTTP handlers */
static esp_err_t pump1_on_handler(httpd_req_t *req);
static esp_err_t pump1_off_handler(httpd_req_t *req);
static esp_err_t pump2_on_handler(httpd_req_t *req);
static esp_err_t pump2_off_handler(httpd_req_t *req);
static esp_err_t pump1_status_handler(httpd_req_t *req);
static esp_err_t pump2_status_handler(httpd_req_t *req);
static esp_err_t api_pump_start_handler(httpd_req_t *req);
static esp_err_t api_pump_stop_handler(httpd_req_t *req);
static esp_err_t api_flow_handler(httpd_req_t *req);
static esp_err_t api_ultrasonic_handler(httpd_req_t *req);
static esp_err_t api_sensors_handler(httpd_req_t *req);
static esp_err_t serve_static_file(httpd_req_t *req);

/* HTTP server functions */
static httpd_handle_t start_server(void);

/* WiFi functions */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void wifi_init_softap(void);

/* Initialization and task functions */
static void init_spiffs(void);
static void startup_task(void *pvParameter);
static void spiffs_task(void *pvParameter);
static void http_task(void *pvParameter);
static void wifi_task(void *pvParameter);
static void pumps_task(void *pvParameter);
static void us_sensor_task(void *pvParameter);

/* ===================================================================
 * PUMP CONTROL IMPLEMENTATION
 * =================================================================== */

/**
 * @brief Initialize pump GPIO pins
 * 
 * Configures GPIO pins for pump control, ensuring they start in the OFF state.
 * Handles both standard and open-drain driver configurations.
 */
static void init_pumps(void)
{
    /* Reset pins to known state, then configure as outputs */
    gpio_reset_pin((gpio_num_t)PUMP1_GPIO);
    gpio_reset_pin((gpio_num_t)PUMP2_GPIO);

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PUMP1_GPIO) | (1ULL << PUMP2_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure pump GPIO: %s", esp_err_to_name(ret));
        return;
    }

    /* Ensure pumps are OFF by default using configured active level */
    int off_level = (PUMP_ACTIVE_LEVEL == 0U) ? 1 : 0;
    
    if ((PUMP_OPEN_DRAIN == 1U) && (PUMP_ACTIVE_LEVEL == 0U)) {
        /* For open-drain low-side drivers, leave pins as inputs (released) */
        (void)gpio_set_direction((gpio_num_t)PUMP1_GPIO, GPIO_MODE_INPUT);
        (void)gpio_set_direction((gpio_num_t)PUMP2_GPIO, GPIO_MODE_INPUT);
    } else {
        (void)gpio_set_level((gpio_num_t)PUMP1_GPIO, off_level);
        (void)gpio_set_level((gpio_num_t)PUMP2_GPIO, off_level);
    }

    ESP_LOGI(TAG, "Pumps initialized (GPIO %u, %u)", (unsigned)PUMP1_GPIO, (unsigned)PUMP2_GPIO);
}

/**
 * @brief Initialize flow sensor GPIO and ISR
 * 
 * Configures the flow sensor input pin and registers the ISR handler
 * for pulse counting.
 */
static void flow_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLOW_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure flow GPIO: %s", esp_err_to_name(ret));
        return;
    }

    ret = gpio_install_isr_service(0U);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = gpio_isr_handler_add(FLOW_GPIO, flow_pulse_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Set pump state (ON/OFF)
 * 
 * @param pump_id ID of pump (1 or 2)
 * @param on True for ON, false for OFF
 * @return ESP_OK on success, ESP_FAIL if invalid pump_id
 */
static esp_err_t pump_set_state(int pump_id, bool on)
{
    uint32_t gpio_num;
    
    if (pump_id == 1) {
        gpio_num = PUMP1_GPIO;
    } else if (pump_id == 2) {
        gpio_num = PUMP2_GPIO;
    } else {
        ESP_LOGE(TAG, "pump_set_state: invalid pump_id %d", pump_id);
        return ESP_FAIL;
    }

    /* Calculate physical GPIO level based on configured active level */
    int active = (PUMP_ACTIVE_LEVEL != 0U) ? 1 : 0;
    int level = on ? active : (1 - active);

    /* Handle open-drain release mode if configured and active-low hardware */
    if ((PUMP_OPEN_DRAIN == 1U) && (PUMP_ACTIVE_LEVEL == 0U)) {
        if (on) {
            /* drive low to turn pump ON */
            (void)gpio_set_direction((gpio_num_t)gpio_num, GPIO_MODE_OUTPUT);
            (void)gpio_set_level((gpio_num_t)gpio_num, 0);
        } else {
            /* release line to turn pump OFF */
            (void)gpio_set_direction((gpio_num_t)gpio_num, GPIO_MODE_INPUT);
        }
    } else {
        /* Re-assert direction before setting level to be robust */
        (void)gpio_set_direction((gpio_num_t)gpio_num, GPIO_MODE_OUTPUT);
        (void)gpio_set_level((gpio_num_t)gpio_num, level);
    }

    int read = gpio_get_level((gpio_num_t)gpio_num);
    if (read != level) {
        ESP_LOGW(TAG, "pump_set_state: mismatch for pump %d gpio %u set=%d read=%d - retrying", pump_id, (unsigned)gpio_num, level, read);
        /* If we tried to set HIGH and it read LOW, try enabling internal pull-up */
        if (level == 1) {
            (void)gpio_set_pull_mode((gpio_num_t)gpio_num, GPIO_PULLUP_ONLY);
            vTaskDelay(pdMS_TO_TICKS(10));
            (void)gpio_set_level((gpio_num_t)gpio_num, level);
            vTaskDelay(pdMS_TO_TICKS(10));
            read = gpio_get_level((gpio_num_t)gpio_num);
            (void)gpio_set_pull_mode((gpio_num_t)gpio_num, GPIO_FLOATING);
        }
        if (read != level) {
            ESP_LOGW(TAG, "pump_set_state: mismatch after retry for pump %d gpio %u set=%d read=%d", pump_id, (unsigned)gpio_num, level, read);
        }
    }
    if (on) {
        /* reset cumulative on pump start */
        s_cumulative_liters = 0.0f;
    }
    
    /* Update software state tracking */
    if (pump_id == 1) {
        s_pump1_state = on;
    } else if (pump_id == 2) {
        s_pump2_state = on;
    }
    
    ESP_LOGI(TAG, "pump_set_state: pump %d -> %s (gpio %u level=%d)", pump_id, on ? "ON" : "OFF", (unsigned)gpio_num, read);
    return ESP_OK;
}

/**
 * @brief Get current pump state
 * 
 * @param pump_id ID of pump (1 or 2)
 * @return True if pump is ON, false if OFF
 */
static bool pump_get_state(int pump_id)
{
    if (pump_id == 1) {
        return s_pump1_state;
    } else if (pump_id == 2) {
        return s_pump2_state;
    } else {
        ESP_LOGE(TAG, "pump_get_state: invalid pump_id %d", pump_id);
        return false;
    }
}


/* ===================================================================
 * HTTP REQUEST HANDLERS (MISRA Compliant)
 * =================================================================== */

/**
 * @brief Handler for turning pump 1 ON
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t pump1_on_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump1_on_handler: request received");
    esp_err_t ret = pump_set_state(1, true);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set pump state");
        return ESP_FAIL;
    }
    
    char resp[64];
    int written = snprintf(resp, sizeof(resp), "{\"pump\":1,\"state\":\"ON\"}");
    if ((written < 0) || ((size_t)written >= sizeof(resp))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/**
 * @brief Handler for turning pump 1 OFF
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t pump1_off_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump1_off_handler: request received");
    esp_err_t ret = pump_set_state(1, false);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set pump state");
        return ESP_FAIL;
    }
    
    char resp[64];
    int written = snprintf(resp, sizeof(resp), "{\"pump\":1,\"state\":\"OFF\"}");
    if ((written < 0) || ((size_t)written >= sizeof(resp))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/**
 * @brief Handler for turning pump 2 ON
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t pump2_on_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump2_on_handler: request received");
    esp_err_t ret = pump_set_state(2, true);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set pump state");
        return ESP_FAIL;
    }
    
    char resp[64];
    int written = snprintf(resp, sizeof(resp), "{\"pump\":2,\"state\":\"ON\"}");
    if ((written < 0) || ((size_t)written >= sizeof(resp))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/**
 * @brief Handler for turning pump 2 OFF
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t pump2_off_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump2_off_handler: request received");
    esp_err_t ret = pump_set_state(2, false);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set pump state");
        return ESP_FAIL;
    }
    
    char resp[64];
    int written = snprintf(resp, sizeof(resp), "{\"pump\":2,\"state\":\"OFF\"}");
    if ((written < 0) || ((size_t)written >= sizeof(resp))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/**
 * @brief Handler for pump 1 status query
 * @param req HTTP request pointer
 * @return ESP_OK on success
 */
static esp_err_t pump1_status_handler(httpd_req_t *req)
{
    bool on = pump_get_state(1);
    char buf[64];
    int written = snprintf(buf, sizeof(buf), "{\"pump\":1,\"state\":\"%s\"}", on ? "ON" : "OFF");
    if ((written < 0) || ((size_t)written >= sizeof(buf))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "pump1_status: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/**
 * @brief Handler for pump 2 status query
 * @param req HTTP request pointer
 * @return ESP_OK on success
 */
static esp_err_t pump2_status_handler(httpd_req_t *req)
{
    bool on = pump_get_state(2);
    char buf[64];
    int written = snprintf(buf, sizeof(buf), "{\"pump\":2,\"state\":\"%s\"}", on ? "ON" : "OFF");
    if ((written < 0) || ((size_t)written >= sizeof(buf))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "pump2_status: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}


/**
 * @brief Handler for API pump start command
 * POST /api/pump/start { "tank_id": 1, "target_amount": 200 }
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t api_pump_start_handler(httpd_req_t *req)
{
    char buf[128];
    if (req->content_len <= 0 || req->content_len > (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }
    
    int ret = httpd_req_recv(req, buf, (size_t)req->content_len);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *tank_id_obj = cJSON_GetObjectItem(json, "tank_id");
    cJSON *target_obj = cJSON_GetObjectItem(json, "target_amount");
    
    if ((tank_id_obj == NULL) || (target_obj == NULL)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing required fields");
        return ESP_FAIL;
    }

    int tank_id = tank_id_obj->valueint;
    int target = target_obj->valueint;
    cJSON_Delete(json);

    ESP_LOGI(TAG, "API Pump Start: tank_id=%d target=%d", tank_id, target);

    esp_err_t err = ESP_OK;
    if (tank_id == 1) {
        err = pump_set_state(1, true);
    } else if (tank_id == 2) {
        err = pump_set_state(2, true);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid tank_id");
        return ESP_FAIL;
    }
    
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set pump state");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}

/**
 * @brief Handler for flow sensor data API
 * GET /api/flow - returns current flow sensor data
 * @param req HTTP request pointer
 * @return ESP_OK on success
 */
static esp_err_t api_flow_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "api_flow_handler: GET /api/flow called");
    float lpm = 0.0f;
    float liters = 0.0f;
    uint32_t pulses = 0U;
    
    flow_get_latest(&lpm, &pulses, &liters);

    char buf[128];
    int written = snprintf(buf, sizeof(buf), 
             "{\"flow_rate_lpm\":%.3f,\"liters\":%.4f,\"pulses\":%u}",
             (double)lpm, (double)liters, (unsigned)pulses);
    
    if ((written < 0) || ((size_t)written >= sizeof(buf))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "api_flow_handler response: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/**
 * @brief Handler for ultrasonic sensor data API
 * GET /api/ultrasonic - returns current ultrasonic distance and calculated litres
 * @param req HTTP request pointer
 * @return ESP_OK on success
 */
static esp_err_t api_ultrasonic_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "api_ultrasonic_handler: GET /api/ultrasonic called");
    char buf[256];
    int written = snprintf(buf, sizeof(buf), "{\"distance_cm\":%u,\"current_level\":%.2f,\"total_received\":%.2f}", 
                          (unsigned)s_ultrasonic_distance_cm, (double)s_ultrasonic_litre, (double)s_cumulative_liters);
    
    if ((written < 0) || ((size_t)written >= sizeof(buf))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Buffer overflow");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "api_ultrasonic_handler response: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/**
 * @brief Handler for sensors availability check
 * GET /api/sensors - minimal endpoint reporting sensor status
 * @param req HTTP request pointer
 * @return ESP_OK on success
 */
static esp_err_t api_sensors_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "api_sensors_handler: request received (sensors disabled)");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"available\":false}");
    return ESP_OK;
}

/**
 * @brief Handler for API pump stop command
 * POST /api/pump/stop { "tank_id": 1 }
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t api_pump_stop_handler(httpd_req_t *req)
{
    char buf[64];
    if (req->content_len <= 0 || req->content_len > (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
        return ESP_FAIL;
    }
    
    int ret = httpd_req_recv(req, buf, (size_t)req->content_len);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *tank_id_obj = cJSON_GetObjectItem(json, "tank_id");
    if (tank_id_obj == NULL) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing tank_id field");
        return ESP_FAIL;
    }
    
    int tank_id = tank_id_obj->valueint;
    cJSON_Delete(json);

    ESP_LOGI(TAG, "API Pump Stop: tank_id=%d", tank_id);

    esp_err_t err = ESP_OK;
    if (tank_id == 1) {
        err = pump_set_state(1, false);
    } else if (tank_id == 2) {
        err = pump_set_state(2, false);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid tank_id");
        return ESP_FAIL;
    }
    
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set pump state");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}


/* ===================================================================
 * SPIFFS FILE SYSTEM INITIALIZATION
 * =================================================================== */

/**
 * @brief Initialize SPIFFS file system
 * 
 * Mounts SPIFFS partition with error checking and reporting.
 */
static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_BASE_PATH,
        .partition_label = NULL,
        .max_files = SPIFFS_MAX_FILES,
        .format_if_mount_failed = false
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS partition not found");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0U;
    size_t used = 0U;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted at %s (total: %u, used: %u)", conf.base_path, (unsigned)total, (unsigned)used);
    }
}


/**
 * @brief Send static files from SPIFFS
 * 
 * Serves static files with proper bounds checking and error handling.
 * Routes API endpoints to specific handlers.
 * 
 * @param req HTTP request pointer
 * @return ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t serve_static_file(httpd_req_t *req)
{
    char filepath[256];
    const char *uri = req->uri;

    if (uri == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No URI");
        return ESP_FAIL;
    }

    /* Don't serve static files for API endpoints - let them be handled by specific handlers */
    if (strncmp(uri, "/api/", 5U) == 0) {
        ESP_LOGW(TAG, "serve_static_file: Rejecting API endpoint %s - should be handled by specific handler", uri);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_FAIL;
    }

    /* Map root to /index.html */
    const char *file_uri = uri;
    if (strcmp(uri, "/") == 0) {
        file_uri = "/index.html";
    }

    size_t base_len = strlen(SPIFFS_BASE_PATH);
    size_t uri_len = strlen(file_uri);
    
    if ((base_len + uri_len + 1U) > sizeof(filepath)) {
        ESP_LOGW(TAG, "Requested URI too long: %s", file_uri);
        httpd_resp_send_err(req, 414, "URI Too Long");
        return ESP_FAIL;
    }

    (void)memcpy(filepath, SPIFFS_BASE_PATH, base_len);
    (void)memcpy(&filepath[base_len], file_uri, uri_len);
    filepath[base_len + uri_len] = '\0';

    FILE *f = fopen(filepath, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "File not found: %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File Not Found");
        return ESP_FAIL;
    }

    static char buffer[HTTP_BUFFER_SIZE];
    size_t r = 0U;
    while ((r = fread(buffer, 1U, sizeof(buffer), f)) > 0U) {
        if (httpd_resp_send_chunk(req, buffer, r) != ESP_OK) {
            (void)fclose(f);
            return ESP_FAIL;
        }
    }

    (void)fclose(f);
    /* signal end of response */
    httpd_resp_send_chunk(req, NULL, 0U);
    return ESP_OK;
}




/* ===================================================================
 * HTTP SERVER INITIALIZATION
 * =================================================================== */

/**
 * @brief Start HTTP server and register all URI handlers
 * 
 * Registers all endpoint handlers and returns server handle.
 * 
 * @return HTTP server handle, or NULL if initialization failed
 */
static httpd_handle_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = HTTP_STACK_SIZE;
    config.max_uri_handlers = HTTP_MAX_HANDLERS;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return NULL;
    }
    
    esp_err_t rc = ESP_OK;
    
    /* Register pump control endpoints */
    httpd_uri_t pump1_on = {
        .uri = "/pump1/on",
        .method = HTTP_GET,
        .handler = pump1_on_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &pump1_on);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump1_on.uri, esp_err_to_name(rc));

    httpd_uri_t pump1_off = {
        .uri = "/pump1/off",
        .method = HTTP_GET,
        .handler = pump1_off_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &pump1_off);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump1_off.uri, esp_err_to_name(rc));

    httpd_uri_t pump2_on = {
        .uri = "/pump2/on",
        .method = HTTP_GET,
        .handler = pump2_on_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &pump2_on);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump2_on.uri, esp_err_to_name(rc));

    httpd_uri_t pump2_off = {
        .uri = "/pump2/off",
        .method = HTTP_GET,
        .handler = pump2_off_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &pump2_off);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump2_off.uri, esp_err_to_name(rc));

    httpd_uri_t pump1_status = {
        .uri = "/pump1/status",
        .method = HTTP_GET,
        .handler = pump1_status_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &pump1_status);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump1_status.uri, esp_err_to_name(rc));

    httpd_uri_t pump2_status = {
        .uri = "/pump2/status",
        .method = HTTP_GET,
        .handler = pump2_status_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &pump2_status);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump2_status.uri, esp_err_to_name(rc));

    /* Register API endpoints */
    httpd_uri_t api_pump_start = {
        .uri = "/api/pump/start",
        .method = HTTP_POST,
        .handler = api_pump_start_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &api_pump_start);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_pump_start.uri, esp_err_to_name(rc));

    httpd_uri_t api_pump_stop = {
        .uri = "/api/pump/stop",
        .method = HTTP_POST,
        .handler = api_pump_stop_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &api_pump_stop);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_pump_stop.uri, esp_err_to_name(rc));

    httpd_uri_t api_sensors = {
        .uri = "/api/sensors",
        .method = HTTP_GET,
        .handler = api_sensors_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &api_sensors);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_sensors.uri, esp_err_to_name(rc));

    httpd_uri_t api_flow = {
        .uri = "/api/flow",
        .method = HTTP_GET,
        .handler = api_flow_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &api_flow);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_flow.uri, esp_err_to_name(rc));

    httpd_uri_t api_ultrasonic = {
        .uri = "/api/ultrasonic",
        .method = HTTP_GET,
        .handler = api_ultrasonic_handler,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &api_ultrasonic);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_ultrasonic.uri, esp_err_to_name(rc));

    /* Register root and catch-all handlers */
    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = serve_static_file,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &root);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register root handler: %s", esp_err_to_name(rc));

    httpd_uri_t static_files = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = serve_static_file,
        .user_ctx = NULL
    };
    rc = httpd_register_uri_handler(server, &static_files);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register catch-all handler: %s", esp_err_to_name(rc));

    ESP_LOGI(TAG, "HTTP server started");
    return server;
}


/* ===================================================================
 * WiFi SOFTAP INITIALIZATION
 * =================================================================== */

/**
 * @brief WiFi event handler for SoftAP mode
 * 
 * Handles station connect and disconnect events.
 * 
 * @param arg Handler argument (unused)
 * @param event_base Event base from esp_event
 * @param event_id Event ID
 * @param event_data Pointer to event-specific data
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg; /* Suppress unused parameter warning */
    
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STACONNECTED)) {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        char mac_str[18];
        int written = snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned)evt->mac[0], (unsigned)evt->mac[1], (unsigned)evt->mac[2], 
                 (unsigned)evt->mac[3], (unsigned)evt->mac[4], (unsigned)evt->mac[5]);
        if ((written >= 0) && ((size_t)written < sizeof(mac_str))) {
            ESP_LOGI(TAG, "station %s join, AID=%d", mac_str, evt->aid);
        }
    } else if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STADISCONNECTED)) {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        char mac_str[18];
        int written = snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 (unsigned)evt->mac[0], (unsigned)evt->mac[1], (unsigned)evt->mac[2], 
                 (unsigned)evt->mac[3], (unsigned)evt->mac[4], (unsigned)evt->mac[5]);
        if ((written >= 0) && ((size_t)written < sizeof(mac_str))) {
            ESP_LOGI(TAG, "station %s leave, AID=%d", mac_str, evt->aid);
        }
    }
}

/**
 * @brief Initialize WiFi in SoftAP mode
 * 
 * Configures and starts WiFi as an access point.
 */
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    (void)esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32S3_AP",
            .ssid_len = 0U,
            .password = "12345678",
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen((const char *)ap_config.ap.password) == 0U) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else if (strlen((const char *)ap_config.ap.password) < 8U) {
        ESP_LOGW(TAG, "Password too short for WPA, switching to OPEN");
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started. SSID:%s  PASS:%s", ap_config.ap.ssid, ap_config.ap.password);
}


// ===================================================================
// STARTUP AND INITIALIZATION TASKS
// ===================================================================

static void spiffs_task(void *pvParameter)
{
    ESP_LOGI(TAG, "spiffs_task: mounting SPIFFS");
    init_spiffs();
    ESP_LOGI(TAG, "spiffs_task: mounted");
    if (spiffs_mounted_sem) {
        xSemaphoreGive(spiffs_mounted_sem);
    }
    vTaskDelete(NULL);
}

static void http_task(void *pvParameter)
{
    /* Wait until SPIFFS is mounted before starting the HTTP server */
    if (spiffs_mounted_sem) {
        ESP_LOGI(TAG, "http_task: waiting for SPIFFS mount");
        xSemaphoreTake(spiffs_mounted_sem, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "http_task: starting HTTP server");
    if (start_server() == NULL) {
        ESP_LOGE(TAG, "http_task: Failed to start HTTP server");
    } else {
        ESP_LOGI(TAG, "http_task: HTTP server started");
    }
    vTaskDelete(NULL);
}

static void wifi_task(void *pvParameter)
{
    ESP_LOGI(TAG, "wifi_task: initializing SoftAP");
    wifi_init_softap();
    ESP_LOGI(TAG, "wifi_task: wifi initialized");
    vTaskDelete(NULL);
}

static void pumps_task(void *pvParameter)
{
    ESP_LOGI(TAG, "pumps_task: initializing pumps");
    init_pumps();
    /* Log stack high water mark to help tune task stack sizes */
    UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "pumps_task: pumps ready (stack high water mark: %u words)", (unsigned)hwm);
    vTaskDelete(NULL);
}

static const char *US_TAG = "ULTRASONIC_APP";
static void us_sensor_task(void *pvParameter)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = GPIO_NUM_33, // change to your TRIG pin
        .echo_pin = GPIO_NUM_32     // change to your ECHO pin
    };

    esp_err_t rc = ultrasonic_init(&sensor);
    if (rc != ESP_OK) {
        ESP_LOGE(US_TAG, "ultrasonic_init failed: %s", esp_err_to_name(rc));
        vTaskDelete(NULL);
        return;
    }
    float litre = 0.0f;
    while (1) {
        uint32_t distance_cm = s_ultrasonic_distance_cm; /* start from last good value */
        /* measure up to US_MEASURE_MAX_CM */
        rc = ultrasonic_measure_cm(&sensor, US_MEASURE_MAX_CM, &distance_cm);
        if (rc == ESP_OK) {
            s_ultrasonic_distance_cm = distance_cm;
            ESP_LOGI(US_TAG, "Distance: %" PRIu32 " cm", distance_cm);
            /* Calculate water volume: (tank_height - distance) * surface_area / 1000 */
            float h = US_TANK_HEIGHT_CM - (float)distance_cm;
            if (h < 0.0f) { h = 0.0f; }
            litre = (h * US_TANK_SURFACE_AREA) / 1000.0f;
            /* clamp to physical capacity */
            float max_l = (US_TANK_HEIGHT_CM * US_TANK_SURFACE_AREA) / 1000.0f;
            if (litre > max_l) { litre = max_l; }
            s_ultrasonic_litre = litre;
            ESP_LOGI(US_TAG, "Litres: %.2f", litre);
        } else {
            /* keep previous reading if measurement failed */
            ESP_LOGW(US_TAG, "ultrasonic_measure_cm error: %s", esp_err_to_name(rc));
        }

        /* Auto-stop at 50% threshold */
        bool distance_at_threshold = ((float)s_ultrasonic_distance_cm <= US_HALF_DISTANCE_CM);
        bool volume_at_threshold = (s_cumulative_liters >= US_HALF_LITERS);
        bool pump1_on = pump_get_state(1);
        bool pump2_on = pump_get_state(2);
        
        if ((distance_at_threshold || volume_at_threshold) && (pump1_on || pump2_on)) {
            ESP_LOGI(US_TAG, "50%% threshold reached (dist: %u cm, cumulative: %.2f L). Stopping pumps.", 
                     (unsigned)s_ultrasonic_distance_cm, (double)s_cumulative_liters);
            (void)pump_set_state(1, false);
            (void)pump_set_state(2, false);
        }

        vTaskDelay(pdMS_TO_TICKS(US_MEASURE_INTERVAL_MS));
    }
    
}

static const char *FM_TAG = "FLOW";

/* ISR: increment atomic pulse counter (IRAM_ATTR for ISR reliability) */
/**
 * @brief ISR: increment atomic pulse counter
 * 
 * Called on rising edge of flow sensor signal.
 * Uses atomic operations for thread-safe counter update.
 * 
 * @param arg ISR argument (unused)
 */
static void IRAM_ATTR flow_pulse_isr(void *arg)
{
    (void)arg; /* Suppress unused parameter warning */
    (void)atomic_fetch_add_explicit(&s_pulse_count, 1U, memory_order_relaxed);
}

/* Measurement task: snapshot-and-reset the atomic counter periodically and compute L/min */
void flow_task(void *pvParameter)
{
    const TickType_t interval = pdMS_TO_TICKS(MEASURE_INTERVAL_MS);
    const float seconds = MEASURE_INTERVAL_MS / 1000.0f;

    while (1) {
        vTaskDelay(interval);

        uint32_t pulses = atomic_exchange_explicit(&s_pulse_count, 0u, memory_order_acq_rel);

        float liters = pulses / PULSES_PER_LITER; // liters during interval
        float lpm = (liters * 60.0f) / seconds;   // liters per minute

        /* Update published state for readers */
        s_flow_state.last_pulses = pulses;
        s_flow_state.last_liters = liters;
        s_flow_state.last_lpm = lpm;
        
        /* Accumulate total received when pump is running */
        if (pump_get_state(1) || pump_get_state(2)) {
            s_cumulative_liters += liters;
        }

        ESP_LOGI(FM_TAG, "pulses=%" PRIu32 " freq=%.2fHz L=%.4f flow=%.3f L/min cumulative=%.4f", pulses, (pulses / seconds), liters, lpm, s_cumulative_liters);
    }
}

/* Accessor: get latest flow values. Because we reset the counter in the task, callers
   should call this after the first measurement interval has elapsed to get meaningful data. */
/**
 * @brief Get latest flow sensor values
 * 
 * Retrieves the most recent snapshot of flow measurements.
 * Values are updated by the flow measurement task.
 * 
 * @param out_lpm Pointer to store flow rate in liters per minute
 * @param out_pulses Pointer to store pulse count in last interval
 * @param out_liters Pointer to store liters in last interval
 */
void flow_get_latest(float *out_lpm, uint32_t *out_pulses, float *out_liters)
{
    if (out_pulses != NULL) {
        *out_pulses = s_flow_state.last_pulses;
    }
    if (out_liters != NULL) {
        *out_liters = s_flow_state.last_liters;
    }
    if (out_lpm != NULL) {
        *out_lpm = s_flow_state.last_lpm;
    }
}

static void startup_task(void *pvParameter)
{
    ESP_LOGI(TAG, "startup_task: begin");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* create semaphore used to notify HTTP task that SPIFFS is mounted */
    spiffs_mounted_sem = xSemaphoreCreateBinary();

    /* Create smaller init tasks so none block the scheduler too long */
    BaseType_t r;
    r = xTaskCreatePinnedToCore(spiffs_task, "spiffs", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    if (r != pdPASS) ESP_LOGW(TAG, "Failed to create spiffs_task");

    r = xTaskCreatePinnedToCore(wifi_task, "wifi", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    if (r != pdPASS) ESP_LOGW(TAG, "Failed to create wifi_task");

    r = xTaskCreatePinnedToCore(http_task, "http", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    if (r != pdPASS) ESP_LOGW(TAG, "Failed to create http_task");

    r = xTaskCreatePinnedToCore(pumps_task, "pumps", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    if (r != pdPASS) ESP_LOGW(TAG, "Failed to create pumps_task");

    r = xTaskCreatePinnedToCore(us_sensor_task, "us_sensor", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    if (r != pdPASS) ESP_LOGW(TAG, "Failed to create us_sensor task");

    xTaskCreatePinnedToCore(flow_task, "flow", 4096, NULL, 5, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "startup_task: all init tasks created, deleting self");
    vTaskDelete(NULL);
}

// ------------------------ APP MAIN ------------------------
void app_main(void)
{
    flow_init();
    BaseType_t r = xTaskCreatePinnedToCore(startup_task, "startup", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create startup task");
    }
}
