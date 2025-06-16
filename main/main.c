#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <sys/time.h>

#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common.h"
#include "adc.h"
#include "nvs_mgmt.h"
#include "wifi.h"
#include "node.h"
#include "rtc.h"
#include "web_server.h"

#define GPIO_WAKEUP_PIN            GPIO_NUM_25
#define LED_ON_BOARD               GPIO_NUM_5
#define DEBOUNCE_COUNTER           50
#define NUMBER_ATTEMPTS            3
#define TAG_MAIN                   "DOOR_SENSOR"

#define ESPNOW_WIFI_CHANNEL        7
#define RETRASMISSION_TIME_MS      50

#define DATA_SENT_SUCCESS          (1 << 0)
#define DATA_SENT_FAILED           (1 << 1)
#define DATA_RECEIVED_SUCCESS      (1 << 2)
#define DATA_RECEIVED_FAILED       (1 << 3)

#define WAKEUP_TIME                10 // In seconds

#define MAC_SIZE                   6

#define NODE_QUEUE_SIZE 4

#define CYCLE_TIME_S    60
#define SLOT_DURATION_S 10

static QueueHandle_t node_queue;
static EventGroupHandle_t xEventGroupDoorSensor;

/* MAC address gateway */
uint8_t dst_mac[MAC_SIZE] = {0x78, 0x42, 0x1C, 0x6A, 0xEF, 0x94};
uint8_t src_mac[MAC_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

RTC_DATA_ATTR bool new_state = 0;
RTC_DATA_ATTR bool old_state = 0;
RTC_DATA_ATTR time_t target_time = 0;

static void print_msg(node_msg_t node_msg) {

    ESP_LOGI(TAG_MAIN, "Cmd: %u", node_msg.header.cmd);
    ESP_LOGI(TAG_MAIN, "Node type: %u", node_msg.header.node);
    ESP_LOGI(TAG_MAIN, "Mac: %02X:%02X:%02X:%02X:%02X:%02X", node_msg.header.mac[0], node_msg.header.mac[1], node_msg.header.mac[2], node_msg.header.mac[3], node_msg.header.mac[4], node_msg.header.mac[5]);
    ESP_LOGI(TAG_MAIN, "ID node: %u", node_msg.header.id_node);
    ESP_LOGI(TAG_MAIN, "ID msg: %u", node_msg.header.id_msg);
    ESP_LOGI(TAG_MAIN, "Name: %s", node_msg.name_node);

    switch(node_msg.header.cmd) {
        case ADD:
            for(uint8_t i = 0; i < 8; i++) {
                ESP_LOGI(TAG_MAIN, "Payload [%u]: %u", i, node_msg.payload[i]);
            }
        break;
        case UPDATE:
        case SYNC:
            ESP_LOGI(TAG_MAIN, "State: %u", node_msg.payload[0]);
            ESP_LOGI(TAG_MAIN, "Battery low detect: %u", node_msg.payload[1]);
        break;
    }

    ESP_LOGI(TAG_MAIN, "CRC16: %d", node_msg.crc);

    return;
}

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    ESP_LOGI(TAG_MAIN, "Receive callback function");

    node_msg_t resp;
    ESP_LOGI(TAG_MAIN, "Receive message from %X:%X:%X:%X:%X:%X", recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2], recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Receive callback arg error");
        return;
    }

    memcpy(&resp, (node_msg_t *)data, sizeof(node_msg_t));
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

/* Send message by espnow protocol */
static bool send_message(uint8_t dest_mac[], node_msg_t msg) {

    esp_err_t err = ESP_FAIL;
    uint8_t num_tentative = NUMBER_ATTEMPTS;
    EventBits_t uxBits;

    xEventGroupClearBits(xEventGroupDoorSensor, DATA_SENT_SUCCESS | DATA_SENT_FAILED);
    do {
        /* Send packet */
        err = esp_now_send(dest_mac, (uint8_t *)&msg, sizeof(msg));
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

    if(!num_tentative)
        return false;

    return true;
}

/* GPIO debounce filter */
static void gpio_debounce_filter(gpio_num_t gpio) {

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

/* Start configuration device */
inline static esp_err_t start_configuration(httpd_handle_t server) {

    if(check_usb_connection()) {

        esp_err_t err = ESP_FAIL;

        ESP_LOGI(TAG_MAIN, "Enter in configuration mode");

        err = wifi_init_softap();
        if (err != ESP_OK)
            return err;
        
        err = start_webserver(server);
        if (err != ESP_OK)
            return err;

        gpio_set_level(LED_ON_BOARD, 0);

        while(1) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        return err;
    }

    return ESP_OK;
}

esp_err_t init_transmission() {

    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer;

    /* Init WiFi station */
    err = init_wifi_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, WiFi not configurated");
        esp_restart();
    }

    /* Init espnow */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, espnow not inited");
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
        ESP_LOGE(TAG_MAIN, "Error, peer not added");
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

    return err;
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

    /* Isolate alls GPIOs unused */
    rtc_gpio_isolate(GPIO_NUM_0);

    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_13);
    rtc_gpio_isolate(GPIO_NUM_14);

    rtc_gpio_isolate(GPIO_NUM_26);
    rtc_gpio_isolate(GPIO_NUM_27);

    rtc_gpio_isolate(GPIO_NUM_32);
    rtc_gpio_isolate(GPIO_NUM_33);
    rtc_gpio_isolate(GPIO_NUM_34);
    rtc_gpio_isolate(GPIO_NUM_35);
    rtc_gpio_isolate(GPIO_NUM_36);
    rtc_gpio_isolate(GPIO_NUM_37);
    rtc_gpio_isolate(GPIO_NUM_38);
    rtc_gpio_isolate(GPIO_NUM_39);

    assert(rtc_gpio_is_valid_gpio(GPIO_WAKEUP_PIN) == true);

    /* Configure GPIO 25 for reed switch */
    ESP_ERROR_CHECK(rtc_gpio_init(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(GPIO_WAKEUP_PIN, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(GPIO_WAKEUP_PIN));

    /* Configure GPIO 5 for led on board */
    const gpio_config_t config_led = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = BIT(LED_ON_BOARD),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    /* Power off led */
    gpio_set_level(LED_ON_BOARD, 1);
    ESP_ERROR_CHECK(gpio_config(&config_led));

    /* Hold on GPIO 25 */
    rtc_gpio_hold_en(GPIO_WAKEUP_PIN);

    return;
}

static void flash_led() {

    for(uint8_t num_flash = 0; num_flash <= NUM_FLASH; num_flash++) {
        gpio_set_level(LED_ON_BOARD, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_ON_BOARD, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return;
}

/* Main program */
void app_main() {

    esp_err_t err = ESP_FAIL;
    char device_name[DEVICE_NAME_SIZE];
    node_msg_t msg;
    httpd_handle_t server = NULL;

    /* Clean message buffer */
    memset(&msg, 0, sizeof(msg));
    memset(device_name, '\0', sizeof(device_name));

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
    #if 0
    err = start_configuration(server);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, configuration mode not started");
        esp_restart();
        return;
    }
    #endif

    init_transmission();

    gpio_debounce_filter(GPIO_WAKEUP_PIN);
    get_device_name(device_name, DEVICE_NAME_SIZE);

    status_door_sensor sdr = {
        .battery_low_detect = check_status_battery(),
        .state = new_state,
    };

    /* Check status configuration */
    switch(1) {
        case UNCONFIGURED_DOOR_SENSOR:
            ESP_LOGW(TAG_MAIN, "Door sensor is not configured. Please connbect usb cable, restart device and configure it");
            while(1) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        break;
        case REGISTRATION_DOOR_SENSOR:
            while(1) {
            ESP_LOGI(TAG_MAIN, "Registration door sensor");

            if(send_message(dest_mac, build_cmd_node_msg(ADD, 1, (esp_random() % 256), src_mac, device_name, &sdr))) {

                set_status_registered(3);
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        break;
        case DEREGISTRATION_DOOR_SENSOR:
            ESP_LOGI(TAG_MAIN, "Deregistration door sensor");

            if(send_message(dest_mac, build_cmd_node_msg(DEL, get_device_id(), (esp_random() % 256), src_mac, device_name, &sdr))) {
                for(uint8_t num_flash = 0; num_flash <= 3; num_flash++) {
                    gpio_set_level(LED_ON_BOARD, 0);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    gpio_set_level(LED_ON_BOARD, 1);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
                set_status_registered(3);
            }
        break;
        case NORMAL_MODE_DOOR_SENSOR:
            ESP_LOGI(TAG_MAIN, "Normal mode");
        break;

    }

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG_MAIN, "Wakeup from timer or GPIO %u", GPIO_WAKEUP_PIN);

            get_device_name(device_name, sizeof(device_name));
            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            msg = build_cmd_node_msg(UPDATE, get_device_id(), (esp_random() % 256), src_mac, device_name, &sdr);
            send_message(dest_mac, msg);
        break;
        default:
            ESP_LOGW(TAG_MAIN, "Warning, source wakeup unknown. First boot");
        break;
    }

    /* Enable wakeup from GPIO 25 (RTC_GPIO6) */
    esp_sleep_enable_gpio_wakeup();

    if (!new_state) {
        ESP_LOGI(TAG_MAIN, "Door open");
        ESP_ERROR_CHECK(rtc_gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_HIGH_LEVEL));
    } else if (new_state) {
        ESP_LOGI(TAG_MAIN, "Door close");
        ESP_ERROR_CHECK(rtc_gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_LOW_LEVEL));
    }

    /* Enable timer wakeup every WAKEUP_TIME seconds */
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(WAKEUP_TIME * 1000000));

    esp_now_deinit();

    deinit_wifi_sta();

    ESP_LOGI(TAG_MAIN, "Going to deep sleep mode");

    vTaskDelay(pdMS_TO_TICKS(5));

    /* Entra in deep sleep */
    esp_deep_sleep_start();

    return;
}