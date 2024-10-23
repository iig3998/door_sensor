#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_err.h"
#include "wifi.h"

#define TAG_WIFI "WIFI"

#define SSID_WIFI_NETWORK "DomoticHouse"
#define PASS_WIFI_NETWORK "D0m0t1cH0use"

/* WiFi event group */
EventGroupHandle_t event_group_wifi;

/* 
 * Start WiFi scan
 */
static void start_wifi_scan() {

    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG_WIFI, "Start WiFi scan ...");

    err = esp_wifi_scan_start(NULL, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, WiFi scan not started");
    }

    return;
}

/*
 * WiFi handler event
 */
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    esp_err_t err = ESP_FAIL;
    uint16_t number = 0;
    wifi_ap_record_t *ap_records = NULL;

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG_WIFI, "Start WiFi driver");
            start_wifi_scan();
        break;

        /* WiFi event scan done */
        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG_WIFI, "Scan WiFi done");

            err = esp_wifi_scan_get_ap_num(&number);
            if (err != ESP_OK){
                ESP_LOGE(TAG_WIFI, "Error to scan WiFI network");
                break;
            }

            if (number != 0) {
                ESP_LOGI(TAG_WIFI, "Scan done. Found %u WiFi Networks", number);
            } else {
                ESP_LOGI(TAG_WIFI, "Scan done. No WiFi Network found");
                start_wifi_scan();
            }

            ap_records = (wifi_ap_record_t *)calloc(number, sizeof(wifi_ap_record_t));
            if (!ap_records) {
                ESP_LOGE(TAG_WIFI, "Error, memory not allocated for WiFi");
                break;
            }

            /* Clear array list WiFi network */
            memset(ap_records, 0, number * sizeof(wifi_ap_record_t));

            err = esp_wifi_scan_get_ap_records(&number, ap_records);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_WIFI, "");
                break;
            }

            ESP_LOGI(TAG_WIFI, "");
            for(uint8_t i = 0; i < number; i++) {
                ESP_LOGI(TAG_WIFI, "WiFi Network name: %s", ap_records[i].ssid);
                ESP_LOGI(TAG_WIFI, "RSSI WiFi Network: %d", ap_records[i].rssi);
                ESP_LOGI(TAG_WIFI, "Channel AP: %d", ap_records[i].primary);
                ESP_LOGI(TAG_WIFI, "Authentication mode AP: %d\n", ap_records[i].authmode);

                if (strncmp((char *)ap_records[i].ssid, SSID_WIFI_NETWORK, strlen(SSID_WIFI_NETWORK)) == 0) {
                    ESP_LOGI(TAG_WIFI, "WiFi network %s found. Start connection ...", SSID_WIFI_NETWORK);
                    esp_wifi_connect();
                    break;
                }
            }
            free(ap_records);
        break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG_WIFI, "WiFi module connect. Start DHCP service ...");
        break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG_WIFI, "WiFi module disconnect");

            /* Clear event group bit */
            xEventGroupClearBits(event_group_wifi, BIT(0));

            /* Clear connection */
            err = esp_wifi_disconnect();
            if (err != ESP_OK) {
                ESP_LOGE(TAG_WIFI, "Error WiFi disconnection");
            }

            start_wifi_scan();

        break;

        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG_WIFI, "Got IP address");

            xEventGroupSetBits(event_group_wifi, BIT(0));
        break;
    }

    return;
}

/* 
 * Print wifi version
 */
void print_wifi_version() {

    ESP_LOGI(TAG_WIFI, "WiFi version: %u.%u.%u", WIFI_MAJOR, WIFI_MINOR, WIFI_PATCH);

    return;
}

/* 
 * Init WiFi
 */
void init_wifi() {

    esp_err_t err = ESP_FAIL;
    esp_netif_t *netif_wifi_sta = NULL;

    ESP_LOGI(TAG_WIFI, "Start WiFi connection");

    event_group_wifi = xEventGroupCreate();

    /* Initialize stack TCP/IP */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, network interface WiFi not init");
        return;
    }

    /* Start handler event function */
    esp_event_loop_create_default();

    netif_wifi_sta = esp_netif_create_default_wifi_sta();
    if (!netif_wifi_sta) {
        ESP_LOGE(TAG_WIFI, "Error, network wifi station not created");
        return;
    }

    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_initiation));

    /* Init handler event function */
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID_WIFI_NETWORK,
            .password = PASS_WIFI_NETWORK,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .bssid_set = false,
            .channel = 0,
        }
    };

    /* Set WiFi configuration */
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WIFI, "Error, WiFI not configured");
        return;
    }

    /* Set station mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK){
        ESP_LOGE(TAG_WIFI, "Error, WiFi mode not set");
        return;
    }

    esp_wifi_set_ps(0);

    /* Start WiFi driver */
    while(esp_wifi_start() != ESP_OK) {
        /* Re-try starts WiFI if fails */
        ESP_LOGW(TAG_WIFI, "Error, re-try starts WiFI");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return;
}

/**/
void wait_wifi_connection_established() {
    
    xEventGroupWaitBits(event_group_wifi, BIT(0), pdFALSE, pdTRUE, portMAX_DELAY);

    /* Wait 3 seconds */
    vTaskDelay(pdMS_TO_TICKS(3000));

    return;
}

/*
 * Get WiFi status connection
 */
uint8_t get_wifi_status_connection() {

    return (xEventGroupWaitBits(event_group_wifi, BIT(0), pdFALSE, pdTRUE, 100 / portTICK_PERIOD_MS) & BIT(0));

}