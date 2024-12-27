#include <string.h>
#include <inttypes.h>

#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_periph.h"
#include "soc/sens_reg.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/adc.h"

//#include "esp_adc_cal.h"
//#include "esp_adc/adc_cali.h"
//#include "esp_adc/adc_oneshot.h"
//#include "esp_adc/adc_continuous.h"

#include "sensor.h"
#include "wifi.h"

#define TAG_MAIN              "MAIN"

#define ID_SENSOR             2
#define NUMBER_ATTEMPTS       3
#define RETRASMISSION_TIME_MS 150
#define DEBOUNCE_COUNTER      3

static uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Read reed switch status with debounce filter */
static bool read_reed_switch() {

    uint8_t gpio_new_state = 1;
    uint8_t gpio_old_state = 1;
    uint8_t counter = 0;

    while (counter < DEBOUNCE_COUNTER) {
        gpio_new_state = gpio_get_level(GPIO_NUM_32);
        if (gpio_new_state != gpio_old_state) {
            counter = 0;
        } else {
            counter ++;
        }
        gpio_old_state = gpio_new_state;
    }

    if (!gpio_new_state)
        return true;

    return false;
}

/* Init ulp program */
static void init_gpio(void) {

    ESP_LOGI(TAG_MAIN, "Init gpio");

    /* Disable GPIO unused */
    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_25);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_26);
    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_27);
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_33);
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_34);
    gpio_set_direction(GPIO_NUM_34, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_35);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_36);
    gpio_set_direction(GPIO_NUM_36, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_37);
    gpio_set_direction(GPIO_NUM_37, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_38);
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_39);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_DISABLE);

    //rtc_gpio_isolate(GPIO_NUM_12);
    //rtc_gpio_isolate(GPIO_NUM_15);

    /* Suppress boot messages */
    esp_deep_sleep_disable_rom_logging();

    /* Configure led */
    gpio_set_level(GPIO_NUM_22, 1);
    gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT);

    /* Configure reed switch */
    gpio_reset_pin(GPIO_NUM_32);
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_NUM_32);
    gpio_pulldown_dis(GPIO_NUM_32);
    
    return;
}

/* Door sensro task */
static void door_sensor_task() {

    esp_err_t err = ESP_FAIL;
    esp_sleep_wakeup_cause_t wakeup_reason;
    node_id_alarm pkt;

    ESP_LOGI(TAG_MAIN, "Start time: %lld", esp_timer_get_time());

    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG_MAIN, "Wakeup from timer");
            break;
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG_MAIN, "Wakeup from gpio");
            break;
        default:
            ESP_LOGW(TAG_MAIN, "Warning, wakeup unkown. It could be the first startup");
        break;
    }

    /* Configure ADC and read value */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

    int adc_data = adc1_get_raw(ADC1_CHANNEL_0);
    float voltage = (adc_data / 4095.0) * 3.3;
    bool battery_low_detect = false;

    if (voltage < 3.15) {
        battery_low_detect = true;
    }

    ESP_LOGI(TAG_MAIN, "ADC data: %d, Voltage: %.2fV", adc_data, voltage);

    /* Send packet */
    memset(&pkt, 0, sizeof(pkt));
    pkt = set_alarm_sensor(ID_SENSOR, read_reed_switch(), battery_low_detect, esp_timer_get_time());

    for(uint8_t i = 0; i < NUMBER_ATTEMPTS; i++) {
        err = esp_now_send(dest_mac, (uint8_t *)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, data not sent");
        }
        vTaskDelay(pdMS_TO_TICKS(RETRASMISSION_TIME_MS));
    }
    gpio_set_level(GPIO_NUM_22, 0);

    ESP_LOGI(TAG_MAIN, "Start deep sleep mode");
    ESP_LOGI(TAG_MAIN, "Start time: %lld", esp_timer_get_time());

    /* Waits 10 seconds */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Start deep sleep mode */
    esp_deep_sleep_start();

    return;
}

#pragma GCC diagnostic ignored "-Wunused-function"
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    if (!mac_addr) {
        ESP_LOGE(TAG_MAIN, "Error, data not sent");
        return;
    }

    return;
}

#pragma GCC diagnostic ignored "-Wunused-function"
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    return;
}

/* Pre app main program */
__attribute__((constructor)) void pre_app_main() {

    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG_MAIN, "Start pre app main");

    /* Init gpio */
    init_gpio();

    return;
}

void test_reed_switch() {

    while(1) {
        ESP_LOGI(TAG_MAIN, "%u", read_reed_switch());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

/* Main program */
void app_main() {
    
    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer;
    
    ESP_LOGI(TAG_MAIN, "Start main program");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, nvs flash not init");
            return;
        }
    }

    /* Enable timer wakeup every 30 seconds */
    err = esp_sleep_enable_timer_wakeup(30 * 1000000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, wakeup from timer not enable");
        return;
    }

    /* Set wakeup gpio */
    err = gpio_wakeup_enable(GPIO_NUM_32, GPIO_INTR_HIGH_LEVEL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, wakeup from gpio not enable");
        return;
    }

    esp_sleep_enable_gpio_wakeup();

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
