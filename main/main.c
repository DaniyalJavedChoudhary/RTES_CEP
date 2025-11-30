#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
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
#include "esp_http_server.h"
#include "esp_netif.h"
#include "cJSON.h"
#include <stdbool.h>



static const char *TAG = "WEB";

// ------------------------ PUMPS GPIO DEFINES ------------------------
#define PUMP1_GPIO  18    // change as needed; remember to wire driver transistor/relay
#define PUMP2_GPIO  19    // change as needed; remember to wire driver transistor/relay
/* Set to 0 if your pump driver is active-low (drive 0 to turn ON).
    Set to 1 if your driver is active-high (drive 1 to turn ON). */
#define PUMP_ACTIVE_LEVEL 0
/* If your pump driver expects the MCU to pull the line low to enable the pump
    and release (float) the line to disable it (common with open-drain drivers),
    set this to 1. Otherwise set to 0 to always drive the pin high/low. */
#define PUMP_OPEN_DRAIN 1


// ------------------------ GPIO INIT FOR DOSING PUMPS ------------------------
static void init_pumps(void)
{
    /* Reset pins to known state, then configure as outputs */
    gpio_reset_pin(PUMP1_GPIO);
    gpio_reset_pin(PUMP2_GPIO);

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PUMP1_GPIO) | (1ULL << PUMP2_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

     /* Ensure pumps are OFF by default using configured active level.
         OFF level is the inverse of the active level. Write twice to stabilize. */
    int off_level = (PUMP_ACTIVE_LEVEL == 0) ? 1 : 0;
    if (PUMP_OPEN_DRAIN && PUMP_ACTIVE_LEVEL == 0) {
        /* For open-drain low-side drivers, leave pins as inputs (released) to keep pumps OFF */
        gpio_set_direction(PUMP1_GPIO, GPIO_MODE_INPUT);
        gpio_set_direction(PUMP2_GPIO, GPIO_MODE_INPUT);
    } else {
        gpio_set_level(PUMP1_GPIO, off_level);
        gpio_set_level(PUMP1_GPIO, off_level);
        gpio_set_level(PUMP2_GPIO, off_level);
        gpio_set_level(PUMP2_GPIO, off_level);
    }

    ESP_LOGI("PUMP", "Pumps initialized (GPIO %d, %d)", PUMP1_GPIO, PUMP2_GPIO);
}

/* Helper: set pump state (true == ON, false == OFF). Returns ESP_OK on success */
static esp_err_t pump_set_state(int pump_id, bool on)
{
    int gpio = (pump_id == 1) ? PUMP1_GPIO : PUMP2_GPIO;
    /* Calculate physical GPIO level based on configured active level */
    int active = PUMP_ACTIVE_LEVEL ? 1 : 0;
    int level = on ? active : (1 - active);

    /* Handle open-drain release mode if configured and active-low hardware */
    if (PUMP_OPEN_DRAIN && PUMP_ACTIVE_LEVEL == 0) {
        if (on) {
            /* drive low to turn pump ON */
            gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
            gpio_set_level(gpio, 0);
        } else {
            /* release line to turn pump OFF */
            gpio_set_direction(gpio, GPIO_MODE_INPUT);
        }
    } else {
        /* Re-assert direction before setting level to be robust */
        gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio, level);
    }

    int read = gpio_get_level(gpio);
    if (read != level) {
        ESP_LOGW(TAG, "pump_set_state: mismatch for pump %d gpio %d set=%d read=%d - retrying", pump_id, gpio, level, read);
        /* If we tried to set the pin HIGH and it read LOW, try enabling internal pull-up and retry once.
           This helps when an external open-drain driver or weak pull-down is present. */
        if (level == 1) {
            gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(gpio, level);
            vTaskDelay(pdMS_TO_TICKS(10));
            read = gpio_get_level(gpio);
            gpio_set_pull_mode(gpio, GPIO_FLOATING);
        }
        if (read != level) {
            ESP_LOGW(TAG, "pump_set_state: still mismatch after retry for pump %d gpio %d set=%d read=%d", pump_id, gpio, level, read);
        }
    }
    ESP_LOGI(TAG, "pump_set_state: pump %d -> %s (gpio %d level=%d)", pump_id, on ? "ON" : "OFF", gpio, read);
    return ESP_OK;
}

static bool pump_get_state(int pump_id)
{
    int gpio = (pump_id == 1) ? PUMP1_GPIO : PUMP2_GPIO;
    int lvl = gpio_get_level(gpio);
    /* Map physical level to logical ON/OFF using configured active level */
    int active = PUMP_ACTIVE_LEVEL ? 1 : 0;
    return (lvl == active);
}


// ------------------------ PUMPS HANDLERS ------------------------
static esp_err_t pump1_on_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump1_on_handler: request received");
    pump_set_state(1, true);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"pump\":1,\"state\":\"ON\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t pump1_off_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump1_off_handler: request received");
    pump_set_state(1, false);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"pump\":1,\"state\":\"OFF\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t pump2_on_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump2_on_handler: request received");
    pump_set_state(2, true);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"pump\":2,\"state\":\"ON\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t pump2_off_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "pump2_off_handler: request received");
    pump_set_state(2, false);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"pump\":2,\"state\":\"OFF\"}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t pump1_status_handler(httpd_req_t *req)
{
    bool on = pump_get_state(1);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"pump\":1,\"state\":\"%s\"}", on ? "ON" : "OFF");
    ESP_LOGI(TAG, "pump1_status: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t pump2_status_handler(httpd_req_t *req)
{
    bool on = pump_get_state(2);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"pump\":2,\"state\":\"%s\"}", on ? "ON" : "OFF");
    ESP_LOGI(TAG, "pump2_status: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}


// POST /api/pump/start { "tank_id": 1, "target_amount": 200 }
static esp_err_t api_pump_start_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, req->content_len);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    int tank_id = cJSON_GetObjectItem(json, "tank_id")->valueint;
    int target = cJSON_GetObjectItem(json, "target_amount")->valueint;
    cJSON_Delete(json);

    ESP_LOGI(TAG, "API Pump Start: tank_id=%d target=%d", tank_id, target);

    // Use helper to set pump state (handles open-drain and active level)
    if (tank_id == 1) pump_set_state(1, true);
    else if (tank_id == 2) pump_set_state(2, true);
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid tank_id");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}

// Minimal /api/sensors handler: tells client that sensor data is not available
static esp_err_t api_sensors_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "api_sensors_handler: request received (sensors disabled)");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"available\":false}");
    return ESP_OK;
}

// POST /api/pump/stop { "tank_id": 1 }
static esp_err_t api_pump_stop_handler(httpd_req_t *req)
{
    char buf[64];
    int ret = httpd_req_recv(req, buf, req->content_len);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    int tank_id = cJSON_GetObjectItem(json, "tank_id")->valueint;
    cJSON_Delete(json);

    ESP_LOGI(TAG, "API Pump Stop: tank_id=%d", tank_id);

    if (tank_id == 1) pump_set_state(1, false);
    else if (tank_id == 2) pump_set_state(2, false);
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid tank_id");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}


// ------------------------ SPIFFS INIT ------------------------
static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
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

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS mounted at %s (total: %u, used: %u)", conf.base_path, (unsigned)total, (unsigned)used);
    }
}


// ------------------------ FILE SENDER ------------------------
static esp_err_t serve_static_file(httpd_req_t *req)
{
    char filepath[256];
    const char *uri = req->uri;

    if (uri == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No URI");
        return ESP_FAIL;
    }

    /* map root to /index.html */
    if (strcmp(uri, "/") == 0) {
        uri = "/index.html";
    }

    size_t base_len = strlen("/spiffs");
    size_t uri_len = strlen(uri);
    if (base_len + uri_len + 1 > sizeof(filepath)) {
        ESP_LOGW(TAG, "Requested URI too long: %s", uri);
        httpd_resp_send_err(req, 414, "URI Too Long");
        return ESP_FAIL;
    }

    memcpy(filepath, "/spiffs", base_len);
    memcpy(filepath + base_len, uri, uri_len);
    filepath[base_len + uri_len] = '\0';

    FILE *f = fopen(filepath, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "File not found: %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File Not Found");
        return ESP_FAIL;
    }

    static char buffer[1024];
    size_t r;
    while ((r = fread(buffer, 1, sizeof(buffer), f)) > 0) {
        if (httpd_resp_send_chunk(req, buffer, r) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }

    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0); /* signal end of response */
    return ESP_OK;
}




// ------------------------ HTTP SERVER ------------------------
static httpd_handle_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096;
    /* increase allowed URI handlers before starting the server */
    config.max_uri_handlers = 32;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return NULL;
    }
    esp_err_t rc;
    // ---- Pump 1 ON ----
    httpd_uri_t pump1_on = {
        .uri = "/pump1/on",
        .method = HTTP_GET,
        .handler = pump1_on_handler
    };
    rc = httpd_register_uri_handler(server, &pump1_on);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump1_on.uri, esp_err_to_name(rc));

    // ---- Pump 1 OFF ----
    httpd_uri_t pump1_off = {
        .uri = "/pump1/off",
        .method = HTTP_GET,
        .handler = pump1_off_handler
    };
    rc = httpd_register_uri_handler(server, &pump1_off);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump1_off.uri, esp_err_to_name(rc));

    // ---- Pump 2 ON ----
    httpd_uri_t pump2_on = {
        .uri = "/pump2/on",
        .method = HTTP_GET,
        .handler = pump2_on_handler
    };
    rc = httpd_register_uri_handler(server, &pump2_on);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump2_on.uri, esp_err_to_name(rc));

    // ---- Pump 2 OFF ----
    httpd_uri_t pump2_off = {
        .uri = "/pump2/off",
        .method = HTTP_GET,
        .handler = pump2_off_handler
    };
    rc = httpd_register_uri_handler(server, &pump2_off);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump2_off.uri, esp_err_to_name(rc));

    // ---- Pump 1 STATUS ----
    httpd_uri_t pump1_status = {
        .uri = "/pump1/status",
        .method = HTTP_GET,
        .handler = pump1_status_handler
    };
    rc = httpd_register_uri_handler(server, &pump1_status);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump1_status.uri, esp_err_to_name(rc));

    // ---- Pump 2 STATUS ----
    httpd_uri_t pump2_status = {
        .uri = "/pump2/status",
        .method = HTTP_GET,
        .handler = pump2_status_handler
    };
    rc = httpd_register_uri_handler(server, &pump2_status);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", pump2_status.uri, esp_err_to_name(rc));

        // ---- API Pump Start ----
    httpd_uri_t api_pump_start = {
        .uri = "/api/pump/start",
        .method = HTTP_POST,
        .handler = api_pump_start_handler
    };
    rc = httpd_register_uri_handler(server, &api_pump_start);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_pump_start.uri, esp_err_to_name(rc));

    // ---- API Pump Stop ----
    httpd_uri_t api_pump_stop = {
        .uri = "/api/pump/stop",
        .method = HTTP_POST,
        .handler = api_pump_stop_handler
    };
    rc = httpd_register_uri_handler(server, &api_pump_stop);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_pump_stop.uri, esp_err_to_name(rc));

    /* Provide a minimal /api/sensors endpoint so client polling doesn't spam 404s */
    httpd_uri_t api_sensors = {
        .uri = "/api/sensors",
        .method = HTTP_GET,
        .handler = api_sensors_handler
    };
    rc = httpd_register_uri_handler(server, &api_sensors);
    if (rc != ESP_OK) ESP_LOGW(TAG, "Failed to register %s: %s", api_sensors.uri, esp_err_to_name(rc));


    /* root handler */
    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = serve_static_file,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root);

    /* catch-all for static files */
    httpd_uri_t static_files = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = serve_static_file,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &static_files);

    ESP_LOGI(TAG, "HTTP server started");
    return server;
}


// ------------------------ WiFi SoftAP ------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
        ESP_LOGI(TAG, "station %s join, AID=%d", mac_str, evt->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 evt->mac[0], evt->mac[1], evt->mac[2], evt->mac[3], evt->mac[4], evt->mac[5]);
        ESP_LOGI(TAG, "station %s leave, AID=%d", mac_str, evt->aid);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32S3_AP",
            .ssid_len = 0,
            .password = "12345678",
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen((const char*)ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else if (strlen((const char*)ap_config.ap.password) < 8) {
        ESP_LOGW(TAG, "Password too short for WPA, switching to OPEN");
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started. SSID:%s  PASS:%s", ap_config.ap.ssid, ap_config.ap.password);
}


// ------------------------ STARTUP TASK ------------------------
/* Semaphore used to signal SPIFFS mount completion */
static SemaphoreHandle_t spiffs_mounted_sem = NULL;

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

    ESP_LOGI(TAG, "startup_task: all init tasks created, deleting self");
    vTaskDelete(NULL);
}

// ------------------------ APP MAIN ------------------------
void app_main(void)
{
    BaseType_t r = xTaskCreatePinnedToCore(startup_task, "startup", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create startup task");
    }
}