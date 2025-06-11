#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "wifi.h"

/* Print wifi version */
void print_wifi_version() {

    ESP_LOGI(TAG_WIFI, "WiFi version: %u.%u.%u", MAJOR_WIFI_VER, MINOR_WIFI_VER, PATCH_WIFI_VER);

    return;
}

/* Init wifi in station mode */
esp_err_t init_wifi_sta() {

    esp_err_t err = ESP_FAIL;

    /* Set wifi init configuration */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi not init");
        return err;
    }

    /* Set wifi storage */
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi storage not set");
        return err;
    }

    /* Set wifi mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi mode not set");
        return err;
    }

    /* Start wifi */
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi not start");
        return err;
    }

    /* Set wifi */
    err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi channel not set");
        return err;
    }

    /* Set wifi protocol */
    err = esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, wifi protocol not set");
        return err;
    }

    esp_wifi_set_promiscuous(false);

    return err;
}

/* Deinit wifi station */
void deinit_wifi_sta() {

    esp_wifi_deinit();

    return;
}