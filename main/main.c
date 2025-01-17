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
#include "sensors.h"

#define GPIO_WAKEUP_PIN       GPIO_NUM_25
#define LED_ONBORAD           GPIO_NUM_22
#define DEBOUNCE_COUNTER      50
#define TAG_MAIN              "DOOR_SENSOR"

#define NUMBER_ATTEMPTS       1
#define ID_SENSOR             2
#define ESPNOW_WIFI_CHANNEL   7
#define RETRASMISSION_TIME_MS 50
#define VREF_STATE_BATTERY    3.15
#define DATA_SENT             (1 << 0)
#define DATA_RECEIVED         (1 << 1)
#define TIMEOUT               300  // milliseconds
#define WAKEUP_TIME           10   // seconds

EventGroupHandle_t xEventGroupDoorSensor;

uint8_t dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

RTC_DATA_ATTR bool new_state = 0;
RTC_DATA_ATTR bool old_state = 0;
RTC_DATA_ATTR bool battery_state = false;

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    ESP_LOGI(TAG_MAIN, "Receive callback function");

    node_id_response resp;
    ESP_LOGI(TAG_MAIN, "Receive message from %X:%X:%X:%X:%X:%X", recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Receive callback arg error");
        return;
    }

    memcpy(&resp, (node_id_response *)data, sizeof(node_id_response));
    if(!resp.ack) {
        ESP_LOGE(TAG_MAIN, "Error, message not received correctly form gateway");
        return;
    }

    xEventGroupClearBits(xEventGroupDoorSensor, DATA_RECEIVED);

    return;
}

/* Send callback function */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    ESP_LOGI(TAG_MAIN, "Send callback function");

    if(status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG_MAIN, "Data sent correctly");
    } else if (status == ESP_NOW_SEND_FAIL){
        ESP_LOGE(TAG_MAIN, "Data not sent correctly");
    }

    xEventGroupClearBits(xEventGroupDoorSensor, DATA_SENT);

    xEventGroupSetBits(xEventGroupDoorSensor, DATA_RECEIVED);

    return;
}

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

/* Init GPIOs */
void init_gpio() {

    /* Isolate all gpio */
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_0);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_2);

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_4);

    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_13);

    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_14);

    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_26);

    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_27);

    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_32);

    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_33);

    gpio_set_direction(GPIO_NUM_34, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_34);

    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_35);

    gpio_set_direction(GPIO_NUM_36, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_36);

    gpio_set_direction(GPIO_NUM_37, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_37);

    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_38);

    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_DISABLE);
    rtc_gpio_hold_en(GPIO_NUM_39);

    //rtc_gpio_isolate(GPIO_NUM_12);
    //rtc_gpio_isolate(GPIO_NUM_15);

    return;
}

/* Pre app main program */
__attribute__((constructor)) void pre_app_main() {

    /* Suppress boot messages */
    esp_deep_sleep_disable_rom_logging();

    xEventGroupDoorSensor = xEventGroupCreate();
    if (!xEventGroupDoorSensor) {
        ESP_LOGE(TAG_MAIN, "Error, event group not created");
        return;
    }

    /* Init gpio */
    init_gpio();

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

    return;
}

static void set_wakeup_source() {

    esp_err_t err = ESP_FAIL;
    uint8_t counter = 0;

    /* Enable wakeup from GPIO 25 */
    if ((counter == 0) && (new_state == 0)) {
        ESP_LOGI(TAG_MAIN, "Door open");
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 1));
    } else if ((counter == 0) && (new_state == 1)) {
        ESP_LOGI(TAG_MAIN, "Door close");
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 0));
    }

    /* Enable timer wakeup every 5 seconds */
    err = esp_sleep_enable_timer_wakeup(WAKEUP_TIME * 1000000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, wakeup from timer not enable");
    }

    return;
}

/* Main program */
void app_main() {

    esp_err_t err = ESP_FAIL;
    //static uint8_t counter = 0;
    static node_id_alarm pkt;
    static esp_now_peer_info_t peer;

    memset(&pkt, 0, sizeof(pkt));

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

            pkt = alarm_sensor_update_pkt(CONFIG_ID_SENSOR, new_state, battery_state, esp_timer_get_time());
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG_MAIN, "Wakeup from GPIO 25");

            gpio_debounce_filter(GPIO_WAKEUP_PIN);

            pkt = alarm_sensor_update_pkt(CONFIG_ID_SENSOR, new_state, battery_state, esp_timer_get_time());

            break;
        default:
            ESP_LOGW(TAG_MAIN, "Warning, wakeup unkown. It could be the first startup");

            pkt = alarm_sensor_add_pkt(CONFIG_ID_SENSOR, esp_timer_get_time());

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
    err = wifi_init_sta();
    if (err != ESP_OK) {
        return;
    }

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
        esp_now_deinit();
        return;
    }

    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered");
    }
    
    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered");
    }

    /* Send packet */
    xEventGroupSetBits(xEventGroupDoorSensor, DATA_SENT);
    gpio_set_level(LED_ONBORAD, 1);

    err = esp_now_send(peer.peer_addr, (uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, data not sent");
    }
    gpio_set_level(LED_ONBORAD, 0);

    /* Wait until the data is sent */
    xEventGroupWaitBits(xEventGroupDoorSensor, DATA_SENT, pdTRUE, pdFALSE, 1000000000);

    int64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < TIMEOUT * 1000) {
        if (!(xEventGroupGetBits(xEventGroupDoorSensor) & DATA_RECEIVED)) {
            ESP_LOGI(TAG_MAIN, "Data received from gateway");
            break;
        }
    }        

    set_wakeup_source();

    ESP_LOGI(TAG_MAIN, "Going to deep sleep mode");

    vTaskDelay(5/portTICK_PERIOD_MS);

    /* Entra in deep sleep */
    esp_deep_sleep_start();

    return;
}