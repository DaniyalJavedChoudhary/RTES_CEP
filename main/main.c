#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
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

static const char *TAG = "WEB";


// ------------------------ SPIFFS INIT ------------------------
static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = true
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

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return NULL;
    }

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


// ------------------------ APP MAIN ------------------------
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_spiffs();
    wifi_init_softap();
    if (start_server() == NULL) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}
