#include <string.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_wifi.h"

#include "wifi.h"
#include "sensor.h"

#define GPIO_WAKEUP_PIN  GPIO_NUM_25
#define GPIO_LED         GPIO_NUM_22
#define DEBOUNCE_COUNTER 50
#define TAG_MAIN         "DoorSensor"

#define NUMBER_ATTEMPTS       3
#define ID_SENSOR             2
#define ESPNOW_WIFI_CHANNEL   7
#define RETRASMISSION_TIME_MS 50
#define VREF_STATE_BATTERY    3.15

uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

RTC_DATA_ATTR bool new_state = 0;
RTC_DATA_ATTR bool old_state = 0;
RTC_DATA_ATTR bool battery_state = false;


/* GPIO debounce filter */
void gpio_debounce_filter(gpio_num_t gpio) {

    uint8_t counter = DEBOUNCE_COUNTER;

    while(counter > 0) {
        new_state = rtc_gpio_get_level(gpio);
        if (new_state != old_state)
            counter = DEBOUNCE_COUNTER;
        else
            counter --;

        old_state = new_state;
    }

    return;
}

void init_gpio() {

    /* Isolate all gpio */
    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_26);
    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_27);
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_DISABLE);

    gpio_reset_pin(GPIO_NUM_32);
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_DISABLE);

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

    return;

}

/* Pre app main program */
__attribute__((constructor)) void pre_app_main() {

    /* Suppress boot messages */
    esp_deep_sleep_disable_rom_logging();

    return;
}

/* Main program */
void app_main() {

    esp_err_t err = ESP_FAIL;
    static uint8_t counter = 0;
    static node_id_alarm pkt;
    static esp_now_peer_info_t peer;

    /* Configure gpio reed switch */
    ESP_ERROR_CHECK(rtc_gpio_init(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(GPIO_WAKEUP_PIN, RTC_GPIO_MODE_INPUT_ONLY));

    /* Configure led on board */
    gpio_set_level(GPIO_NUM_22, 1);
    ESP_ERROR_CHECK(gpio_pullup_en(GPIO_NUM_22));
    ESP_ERROR_CHECK(gpio_pulldown_dis(GPIO_NUM_22));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT));

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG_MAIN, "Wakeup from timer");

            /* Configure ADC and read value */
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

            int adc_data = adc1_get_raw(ADC1_CHANNEL_0);
            float voltage = (adc_data / 4095.0) * 3.3;

            if (voltage <= VREF_STATE_BATTERY) {
                battery_state = true;
            }

            ESP_LOGI(TAG_MAIN, "ADC data: %d, Voltage: %.2fV", adc_data, voltage);
            break;
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG_MAIN, "Wakeup from GPIO 25");

            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            break;
        default:
            ESP_LOGW(TAG_MAIN, "Warning, wakeup unkown. It could be the first startup");

            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            break;
    }

    /* Init NVS flash */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Init WiFi station */
    wifi_init_sta();

    /* Init espnow */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, ESPNOW not init");
        return;
    }

    /* Add peer to list */
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    memcpy(peer.peer_addr, dest_mac, ESP_NOW_ETH_ALEN);
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, peer not add");
        return;
    }

    /* Send packet */
    memset(&pkt, 0, sizeof(pkt));
    pkt = set_alarm_sensor(ID_SENSOR, new_state, battery_state, esp_timer_get_time());

    gpio_set_level(GPIO_NUM_22, 1);

    for(uint8_t i = 0; i < NUMBER_ATTEMPTS; i++) {
        err = esp_now_send(peer.peer_addr, (uint8_t *)&pkt, sizeof(pkt));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, data not sent");
        }
        vTaskDelay(pdMS_TO_TICKS(RETRASMISSION_TIME_MS * ID_SENSOR));
    }

    gpio_set_level(GPIO_NUM_22, 0);

    /* Enable wakeup from GPIO 25 */
    if ((counter == 0) && (new_state == 0)) {
        ESP_LOGI(TAG_MAIN, "Door open");
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 1));
    } else if ((counter == 0) && (new_state == 1)) {
        ESP_LOGI(TAG_MAIN, "Door close");
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 0));
    }

    /* Enable timer wakeup every 5 seconds */
    err = esp_sleep_enable_timer_wakeup(5 * 1000000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, wakeup from timer not enable");
    }

    ESP_LOGI(TAG_MAIN, "Going to deep sleep mode");

    vTaskDelay(5/portTICK_PERIOD_MS);

    /* Entra in deep sleep */
    esp_deep_sleep_start();

    return;
}