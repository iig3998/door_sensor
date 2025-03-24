#include <string.h>
#include <assert.h>

#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_wifi.h"

#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"

#include "driver/rtc_io.h"
#include "driver/adc.h"

#include "esp_adc/adc_oneshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_mgmt.h"
#include "common.h"
#include "header.h"
#include "wifi.h"
#include "sensor.h"
#include "web_server.h"

#define GPIO_WAKEUP_PIN       GPIO_NUM_25 //RTC_GPIO6
#define LED_ON_BOARD          GPIO_NUM_5

#define DEBOUNCE_COUNTER      50
#define TAG_MAIN              "DOOR_SENSOR"

#define NUMBER_ATTEMPTS       3

#define ESPNOW_WIFI_CHANNEL   7
#define RETRASMISSION_TIME_MS 50
#define VREF_STATE_BATTERY    3.15
#define DATA_SENT_SUCCESS     (1 << 0)
#define DATA_SENT_FAILED      (1 << 1)

#define DATA_RECEIVED         (1 << 1)
#define WAKEUP_TIME           10   // seconds

#define DEVICE_NAME_SIZE      15
#define MAC_SIZE              6

EventGroupHandle_t xEventGroupDoorSensor;

uint8_t dest_mac[6] = {0x78, 0x42, 0x1C, 0x6A, 0xEF, 0x94};
uint8_t src_mac[6] = {0, 0, 0, 0, 0, 0};

RTC_DATA_ATTR bool new_state = 0;
RTC_DATA_ATTR bool old_state = 0;
RTC_DATA_ATTR bool battery_state = false;

#ifdef RECEIVE_CALLBACK_FUNCTION
/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    ESP_LOGI(TAG_MAIN, "Receive callback function");

    node_sensor_msg_t resp;
    ESP_LOGI(TAG_MAIN, "Receive message from %X:%X:%X:%X:%X:%X", recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Receive callback arg error");
        return;
    }

    memcpy(&resp, (node_sensor_msg_t *)data, sizeof(node_sensor_msg_t));
    switch(resp.header.cmd) {
        case NACK:
            ESP_LOGE(TAG_MAIN, "Error, message not received correctly from gateway");
            return;
        break;
        case ACK:
            ESP_LOGI(TAG_MAIN, "Message received correctly from gateway");
        break;
        default:
            ESP_LOGW(TAG_MAIN, "Warning, command not valid");
        break;
    }

    xEventGroupSetBits(xEventGroupDoorSensor, DATA_RECEIVED);

    return;
}
#endif

/* Send callback function */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {

    ESP_LOGI(TAG_MAIN, "Send callback function");

    if(!mac_addr) {
        ESP_LOGE(TAG_MAIN, "Error, mac address is empty");
        return;
    }

    if(status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG_MAIN, "Data sent correctly");
        xEventGroupSetBits(xEventGroupDoorSensor, DATA_SENT_SUCCESS);
    } else if (status == ESP_NOW_SEND_FAIL){
        ESP_LOGE(TAG_MAIN, "Data not sent correctly");
        xEventGroupSetBits(xEventGroupDoorSensor, DATA_SENT_FAILED);
    }

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
static void init_gpio() {

    /* Disable alls GPIOs */
    rtc_gpio_isolate(GPIO_NUM_0);
    rtc_gpio_isolate(GPIO_NUM_1);
    rtc_gpio_isolate(GPIO_NUM_3);
    rtc_gpio_isolate(GPIO_NUM_2);
    rtc_gpio_isolate(GPIO_NUM_6);
    rtc_gpio_isolate(GPIO_NUM_7);
    rtc_gpio_isolate(GPIO_NUM_8);
    rtc_gpio_isolate(GPIO_NUM_9);
    rtc_gpio_isolate(GPIO_NUM_10);
    rtc_gpio_isolate(GPIO_NUM_11);

    rtc_gpio_isolate(GPIO_NUM_12);

    return;
}

inline static void set_wakeup_source() {

    uint8_t counter = 0;

    /* Enable wakeup from GPIO 25 */
    if ((counter == 0) && (new_state == 0)) {
        ESP_LOGI(TAG_MAIN, "Door open");
        ESP_ERROR_CHECK(rtc_gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_HIGH_LEVEL));
    } else if ((counter == 0) && (new_state == 1)) {
        ESP_LOGI(TAG_MAIN, "Door close");
        ESP_ERROR_CHECK(rtc_gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_LOW_LEVEL));
    }

    /* Enable timer wakeup every WAKEUP_TIME seconds */
    /*err = esp_sleep_enable_timer_wakeup(WAKEUP_TIME * 1000000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, wakeup from timer not enable");
    }*/

    return;
}

/* Start configuration device */
inline static bool start_configuration() {

    if(!is_device_configured()) {

        esp_err_t err = ESP_FAIL;

        ESP_LOGI(TAG_MAIN, "Enter in configuration mode");

        /* Init web server */
        err = init_webserver();
        if(err != ESP_OK) {
            return false;
        }

        /* Start web server */
        err = start_webserver();
        if(err != ESP_OK) {
            return false;
        }

        while(!is_device_configured()) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        ESP_LOGI(TAG_MAIN, "Exit from configuration mode");

        /* Stop web server */
        stop_webserver();

        return true;
    }

    return false;
}

/* Pre app main program */
__attribute__((constructor)) void pre_app_main() {

    esp_err_t err = ESP_FAIL;

    /* Suppress boot messages */
    esp_deep_sleep_disable_rom_logging();

    xEventGroupDoorSensor = xEventGroupCreate();
    if (!xEventGroupDoorSensor) {
        ESP_LOGE(TAG_MAIN, "Error, event group not created");
        esp_restart();
    }

    /* Read MAC address */
    err = esp_read_mac(src_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address not read");
        esp_restart();
    }

    /* Init gpio */
    init_gpio();

    assert(esp_sleep_is_valid_wakeup_gpio(GPIO_WAKEUP_PIN) == true);
    
    assert(rtc_gpio_is_valid_gpio(GPIO_WAKEUP_PIN) == true);

    /* Configure GPIO 25 for reed switch */
    rtc_gpio_init(GPIO_WAKEUP_PIN);
    rtc_gpio_set_direction(GPIO_WAKEUP_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_dis(GPIO_WAKEUP_PIN);
    rtc_gpio_pulldown_en(GPIO_WAKEUP_PIN);

    //esp_err_t rtc_gpio_set_direction_in_sleep(gpio_num_t gpio_num, rtc_gpio_mode_t mode)

    /* Configure GPIO 5 for led on board */
    const gpio_config_t config_led = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = BIT(LED_ON_BOARD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&config_led));

    /* Hold on GPIO 25 */
    rtc_gpio_hold_en(GPIO_WAKEUP_PIN);

    return;
}


#if 0
void app_main() {

    int adc_data = 0;
    adc_oneshot_unit_handle_t adc2_handle;
    // IO35
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc2_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_6,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));

    assert(esp_sleep_is_valid_wakeup_gpio(GPIO_WAKEUP_PIN) == true);

    while(1) {

            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_data));
            ESP_LOGI(TAG_MAIN, "ADC data: %d", adc_data);
            vTaskDelay(pdMS_TO_TICKS(1000));

            ESP_LOGI(TAG_MAIN, "GPIO4: %u", gpio_get_level(GPIO_NUM_4));
        }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}
#endif

/* Main program */
void app_main() {

    esp_err_t err = ESP_FAIL;
    uint8_t num_tentative = NUMBER_ATTEMPTS;
    char device_name[DEVICE_NAME_SIZE];
    EventBits_t uxBits;
    node_sensor_msg_t msg;
    esp_now_peer_info_t peer;

    /* Clean message buffer */
    memset(&msg, 0, sizeof(msg));

    /* Init NVS flash */
    err = init_nvs();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, NVS not inited");
        esp_restart();
    }

    /* Init network interface */
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, network interface not init");
        esp_restart();
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, event loop not init");
        esp_restart();
    }

    /* Configure device */
    if(start_configuration()) {
        ESP_LOGI(TAG_MAIN, "Door sensor already configurated!");
    }

    /* Reads device name */
    get_device_name(device_name, sizeof(device_name));
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG_MAIN, "Wakeup from timer");

            /* Configure ADC and read value */
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);

            int adc_data = adc1_get_raw(ADC1_CHANNEL_3);
            float voltage = (adc_data / 4095.0) * 3.3;

            if (voltage <= VREF_STATE_BATTERY) {
                battery_state = true;
            }

            ESP_LOGI(TAG_MAIN, "ADC data: %d, Voltage: %.2fV", adc_data, voltage);

            msg = build_request_update_sensor_msg(get_device_id(), (esp_random() % 256), src_mac, device_name, new_state, battery_state);

            break;
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG_MAIN, "Wakeup from GPIO %u", GPIO_WAKEUP_PIN);

            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            msg = build_request_update_sensor_msg(get_device_id(), (esp_random() % 256), src_mac, device_name, new_state, battery_state);

            break;
        default:
            ESP_LOGD(TAG_MAIN, "Warning, source wakeup unknown");

            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            break;
    }

    /* Init WiFi station */
    err = init_wifi_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, WiFi not configurated");
        esp_restart();
    }

    /* Init espnow */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, ESPNOW not inited");
        esp_restart();
    }

    /* Add peer to list */
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    memcpy(peer.peer_addr, dest_mac, MAC_SIZE);
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, peer not add");
        esp_restart();
    }

    /* Register send callback function */
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered");
        esp_restart();
    }
    
    #ifdef RECEIVE_CALLBACK_FUNCTION
    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered");
        esp_restart();
    }
    xEventGroupClearBits(xEventGroupDoorSensor, DATA_RECEIVED);
    #endif

    gpio_set_level(LED_ON_BOARD, 0);

    xEventGroupClearBits(xEventGroupDoorSensor, DATA_SENT_SUCCESS | DATA_SENT_FAILED);
    do {
        /* Send packet */
        err = esp_now_send(peer.peer_addr, (uint8_t *)&msg, sizeof(msg));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error, data not sent ... retry");
            num_tentative = 0;
        }
        /* Wait until the data is sent */
        uxBits = xEventGroupWaitBits(xEventGroupDoorSensor, DATA_SENT_SUCCESS | DATA_SENT_FAILED, pdTRUE, pdFALSE, portMAX_DELAY);
        num_tentative--;
        vTaskDelay(pdMS_TO_TICKS(RETRASMISSION_TIME_MS));
    }
    while((uxBits & DATA_SENT_FAILED) && num_tentative > 0);

    gpio_set_level(LED_ON_BOARD, 1);

    /* Set wakeup source */
    set_wakeup_source();

    esp_now_deinit();

    deinit_wifi_sta();

    ESP_LOGI(TAG_MAIN, "Going to deep sleep mode");

    vTaskDelay(pdMS_TO_TICKS(5));

    /* Entra in deep sleep */
    esp_deep_sleep_start();

    return;
}

#if 0
#define WAKEUP_GPIO GPIO_NUM_4

void app_main(void) {
    ESP_LOGI("DEEP_SLEEP", "Configurazione wake-up sul GPIO %d", WAKEUP_GPIO);

    /*
    if ((gpio_get_level(WAKEUP_GPIO) == 0)) {
        ESP_LOGI(TAG_MAIN, "Door open");
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1 << GPIO_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH));
    } else if ((gpio_get_level(WAKEUP_GPIO) == 1)) {
        ESP_LOGI(TAG_MAIN, "Door close");
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(1 << GPIO_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_LOW));
    }
    */

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI("DEEP_SLEEP", "Entrando in deep sleep...");
    esp_deep_sleep_start();
}
#endif