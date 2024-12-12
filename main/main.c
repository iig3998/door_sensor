#include <string.h>
#include <inttypes.h>

#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_pm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_periph.h"
#include "soc/sens_reg.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"

#include "ulp.h"
#include "ulp_main.h"

#include "sensor.h"
#include "wifi.h"

#define TAG_MAIN              "MAIN"

#define ID_SENSOR             2
#define NUMBER_ATTEMPTS       3
#define RETRASMISSION_TIME_MS 150

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Read status sensor */
static bool read_status_sensor() {

    if (ulp_prev_state & UINT16_MAX)
        return true;

    return false;
}

/* Init ulp program */
static void init_ulp_program(void) {

    /* GPIO used for pulse counting */
    uint8_t gpio_num = GPIO_NUM_32;

    ESP_LOGI(TAG_MAIN, "Init ulp program");

    /* Define ulp variables */
    ulp_prev_state = 0;
    ulp_debounce_counter = 5;
    ulp_debounce_max_count = 5;

    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, ulp binary not load");
        return;
    }

    /* Warning: ESP32 pins 34, 35, 36 and 39 are input only pins and do not have internal pullup resistors */
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for read GPIO must be an RTC IO");

    ulp_gpio_number = rtc_io_number_get(gpio_num);
    ESP_LOGI(TAG_MAIN, "RTC gpio number: %lu", ulp_gpio_number);

    /* Initialize selected GPIO as RTC IO, enable input, enable pullup and disable pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_en(gpio_num);

    /* Reset all gpio */
    gpio_reset_pin(GPIO_NUM_0);
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_4);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_reset_pin(GPIO_NUM_14);
    gpio_reset_pin(GPIO_NUM_25);
    gpio_reset_pin(GPIO_NUM_26);
    gpio_reset_pin(GPIO_NUM_27);
    gpio_reset_pin(GPIO_NUM_32);
    gpio_reset_pin(GPIO_NUM_33);
    gpio_reset_pin(GPIO_NUM_34);
    gpio_reset_pin(GPIO_NUM_35);
    gpio_reset_pin(GPIO_NUM_36);
    gpio_reset_pin(GPIO_NUM_37);
    gpio_reset_pin(GPIO_NUM_38);
    gpio_reset_pin(GPIO_NUM_39);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors on modules which have these (e.g. ESP32-WROVER)
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);

    /* Suppress boot messages */
    //esp_deep_sleep_disable_rom_logging();

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, ulp program not started");
    }

    return;
}

/* Door sensro task */
static void door_sensor_task() {

    esp_err_t err = ESP_FAIL;
    esp_sleep_wakeup_cause_t wakeup_reason;
    node_id_alarm pkt;

    while(1) {

        ESP_LOGI(TAG_MAIN, "Start time: %lld", esp_timer_get_time());

        wakeup_reason = esp_sleep_get_wakeup_cause();
        switch (wakeup_reason) {
            case ESP_SLEEP_WAKEUP_ULP:
                ESP_LOGI(TAG_MAIN, "Wakeup from ulp");
                break;
            case ESP_SLEEP_WAKEUP_TIMER:
                ESP_LOGI(TAG_MAIN, "Wakeup from timer");
                break;
            default:
                ESP_LOGW(TAG_MAIN, "Warning, wakeup unkown. It could be the first startup");
            break;
        }

        /* Send packet */
        memset(&pkt, 0, sizeof(pkt));
        pkt = set_alarm_sensor(ID_SENSOR, read_status_sensor, esp_timer_get_time());

        for(uint8_t i = 0; i < NUMBER_ATTEMPTS; i++) {
            esp_now_send(dest_mac, (uint8_t *)&pkt, sizeof(pkt));
            if (err != ESP_OK) {
                ESP_LOGE(TAG_MAIN, "Error, data not sent");
            }
            vTaskDelay(pdMS_TO_TICKS(RETRASMISSION_TIME_MS));
        }

        /* Enable ULP wakeup */
        err = esp_sleep_enable_ulp_wakeup();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, wakeup from ulp not enable");
        }

        /* Enable timer wakeup */
        err = esp_sleep_enable_timer_wakeup(10 * 1000000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, wakeup from timer not enable");
        }

        ESP_LOGI(TAG_MAIN, "Start deep sleep mode");

        ESP_LOGI(TAG_MAIN, "Start time: %lld", esp_timer_get_time());

        /* Start deep sleep mode */
        esp_deep_sleep_start();
    }
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    if (!mac_addr) {
        ESP_LOGE(TAG_MAIN, "Error, data not sent");
        return;
    }

    return;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    return;
}

/* Pre app main program */
__attribute__((constructor)) void pre_app_main() {

    ESP_LOGI(TAG_MAIN, "Start pre app main");

    /* Init ulp program */
    init_ulp_program();

    return;
}

/* Main program */
void app_main() {
    
    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer;
    
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, nvs flash not init");
            return;
        }
    }

    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 40,
        .light_sleep_enable = true
    };
    //esp_pm_configure(&pm_config);

    /* Init WiFi espnow */
    err = wifi_init_sta();
    if (err != ESP_OK) {
        return;
    }

    /* Init espnow module */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGI(TAG_MAIN, "Error, espnow module not init");
        return;
    }

    /* Add peer information to peer list */
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, dest_mac, ESP_NOW_ETH_ALEN);
    
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, peer not added");
        return;
    }

    xTaskCreate(door_sensor_task, "door_sensor", 1024 * 2, NULL, 0, NULL);

    return;
}
