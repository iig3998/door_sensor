#include <stdio.h>
#include <string.h>

//#include <freertos/FreeRTOS.h>
//#include <freertos/event_groups.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"

#include "wifi.h"

/* Print WiFi version */
void print_wifi_version() {

    ESP_LOGI(TAG_WIFI, "WiFi version: %u.%u.%u", WIFI_MAJOR, WIFI_MINOR, WIFI_PATCH);

    return;
}

/* */
esp_err_t wifi_init_sta() {

    esp_err_t err = ESP_FAIL;

    /* Init network interface */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, network interface not init");
        return ESP_FAIL;
    }

    /*
    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, event loop not init");
        return ESP_FAIL;
    }*/

    /* Set wifi init configuration */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi not init");
        return ESP_FAIL;
    }

    /* Set wifi storage */
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi storage not set");
        return ESP_FAIL;
    }

    /* Set wifi mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi mode not set");
        return ESP_FAIL;
    }

    /* Start wifi */
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi not start");
        return ESP_FAIL;
    }

    /* Set wifi */
    err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi channel not set");
        return ESP_FAIL;
    }

    /* Set wifi protocol */
    err = esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi protocol not set");
        return ESP_FAIL;
    }

    return ESP_OK;
}