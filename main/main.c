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

    if (!recv_info->src_addr || !data || len <= 0) {
        ESP_LOGE(TAG_MAIN, "Error, receive callback arg");
        return;
    }

    node_msg_t msg;
    memcpy(&msg, data, sizeof(node_msg_t));
    print_msg(msg);

    if (xQueueSend(node_queue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
        ESP_LOGW(TAG_MAIN, "Warning, queue is full, discard message");
        xEventGroupSetBits(xEventGroupDoorSensor, DATA_RECEIVED_FAILED);
    } else {
        xEventGroupSetBits(xEventGroupDoorSensor, DATA_RECEIVED_SUCCESS);
    }

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
static bool send_message(uint8_t dst_mac[], node_msg_t msg) {

    esp_err_t err = ESP_FAIL;
    uint8_t num_tentative = NUMBER_ATTEMPTS;
    EventBits_t uxBits;

    xEventGroupClearBits(xEventGroupDoorSensor, DATA_SENT_SUCCESS | DATA_SENT_FAILED);
    do {
        /* Send packet */
        err = esp_now_send(dst_mac, (uint8_t *)&msg, sizeof(msg));
        if (err != ESP_OK) {
            ESP_LOGE(TAG_MAIN, "Error send: %s", esp_err_to_name(err));
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

/* Calculate awake time */
static time_t calculate_awake_time_in_slot(uint8_t slot_index) {

    time_t now;
    time_t offset_in_cycle;
    time_t slot_start;
    time_t slot_end;
    time_t delta;

    time(&now);

    delta = now - target_time;

    if (delta < 0)
        delta = 0;

    offset_in_cycle = delta % CYCLE_TIME_S;
    slot_start = (slot_index - 1) * SLOT_DURATION_S;
    slot_end = slot_start + SLOT_DURATION_S;

    if (offset_in_cycle < slot_start) {
        return slot_start - offset_in_cycle;
    } else if (offset_in_cycle >= slot_start && offset_in_cycle < slot_end) {
        return slot_end - offset_in_cycle;
    }

    return CYCLE_TIME_S - offset_in_cycle + slot_start;
}

/* Calculates the sleep duration needed to wake up at the next available time slot */
static time_t enter_deep_sleep_until_slot(uint8_t slot_index) {

    uint32_t cycles_passed = 0;
    time_t sleep_time_s = 0;
    time_t delta = 0;
    time_t first_wakeup = 0;
    time_t now = 0;

    time(&now);

    delta = now - target_time;
    if (delta > 0) {
        cycles_passed = delta / CYCLE_TIME_S;
    }

    //int slot_offset = (slot_index - 1) * SLOT_DURATION_S;
    first_wakeup = target_time + (cycles_passed + 1) * CYCLE_TIME_S + ((slot_index - 1) * SLOT_DURATION_S);
    sleep_time_s = first_wakeup - now;

    if (sleep_time_s <= 0) {
        ESP_LOGW(TAG_MAIN, "Sleep time negativo o zero, sveglia immediata");
        return -1;
    }

    ESP_LOGI(TAG_MAIN, "Sleep per %lld secondi. Sensore slot %u si sveglia a %lld", sleep_time_s, slot_index, first_wakeup);

    return sleep_time_s;
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

/* Power on led */
static void power_on_led() {

    gpio_set_level(LED_ON_BOARD, 0);

    return;
}

/* Power off led */
static void power_off_led() {

    gpio_set_level(LED_ON_BOARD, 1);

    return;
}

/* Toggle led */
static void toggle_led(const uint8_t num_flash, const uint16_t time_flash) {

    for(uint8_t i = 0; i <= num_flash; i++) {
        gpio_set_level(LED_ON_BOARD, 0);
        vTaskDelay(pdMS_TO_TICKS(time_flash));
        gpio_set_level(LED_ON_BOARD, 1);
        vTaskDelay(pdMS_TO_TICKS(time_flash));
    }

    return;
}

/* Start configuration device */
inline static esp_err_t start_configuration(httpd_handle_t server) {

    if(!check_usb_connection()) {

        esp_err_t err = ESP_FAIL;

        ESP_LOGI(TAG_MAIN, "Enter in configuration mode");

        err = wifi_init_softap();
        if (err != ESP_OK)
            return err;

        err = start_webserver(server);
        if (err != ESP_OK)
            return err;

        power_on_led();

        while(1) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        power_off_led();

        return err;
    }

    return ESP_OK;
}

/* Init trasmission */
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

    memcpy(peer.peer_addr, dst_mac, MAC_SIZE);
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

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered");
        esp_restart();
    }

    return err;
}

/* Enter in deep sleep mode */
static void enter_in_deep_sleep_mode(time_t time_sleep) {

    ESP_LOGI(TAG_MAIN, "Enter in deep sleep mode");

    /* Enable wakeup from GPIO 25 (RTC_GPIO_6) */
    esp_sleep_enable_gpio_wakeup();

    if (!new_state) {
        ESP_LOGI(TAG_MAIN, "Door open");
        ESP_ERROR_CHECK(rtc_gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_HIGH_LEVEL));
    } else if (new_state) {
        ESP_LOGI(TAG_MAIN, "Door close");
        ESP_ERROR_CHECK(rtc_gpio_wakeup_enable(GPIO_WAKEUP_PIN, GPIO_INTR_LOW_LEVEL));
    }

    /* Enable timer wakeup every WAKEUP_TIME seconds */
    esp_sleep_enable_timer_wakeup(time_sleep * 1000000ULL);

    esp_now_deinit();

    deinit_wifi_sta();

    vTaskDelay(pdMS_TO_TICKS(10));

    esp_deep_sleep_start();
}

/* Configuration task */
static void configuration_task(void *arg) {

    esp_err_t err = ESP_FAIL;
    httpd_handle_t server = NULL;

    ESP_LOGI(TAG_MAIN, "Enter in configuration mode");

    err = wifi_init_softap();
    if (err != ESP_OK)
        return;

    err = start_webserver(server);
    if (err != ESP_OK)
        return;

    gpio_set_level(LED_ON_BOARD, 0);

    while(!check_usb_connection()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    gpio_set_level(LED_ON_BOARD, 1);

    stop_webserver(server);

    return;
}

/* Normal mode task */
static void normal_mode_task(void *arg) {

    static time_t t_start = 0;
    static time_t t_end = 0;
    static time_t time_sleep = 60;
    static node_msg_t msg;

    /* Clean message buffer */
    memset(&msg, 0, sizeof(msg));

    /* Init trasmission */
    init_transmission();

    switch(get_status_registration()) {
        /* Unregistration mode */
        case 0:
            ESP_LOGI(TAG_MAIN, "Unregistration mode. Please registrate device. Restart from 3 seconds");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
        break;
        /* Registration mode */
        case 1:
            ESP_LOGI(TAG_MAIN, "Registration mode");
            status_node sdr = {
                .battery_low_detect = check_status_battery(),
                .state = new_state,
            };

            while(get_status_registration() != 2) {
                msg = build_node_msg(ADD, get_device_id(), SENSOR, (esp_random() % 256), src_mac, get_device_name(), &sdr);
                if(!send_message(dst_mac, msg)) {
                    ESP_LOGW(TAG_MAIN, "Warning, data not sent. Ensure that the gateway is powered on");

                    /* Read variable for new tentative */
                    sdr.battery_low_detect = check_status_battery();
                    sdr.state = new_state;
                }

                /* Retry msg registration if not receive response message after 20 seconds */
                memset(&msg, 0, sizeof(msg));
                if (xQueueReceive(node_queue, &msg, pdMS_TO_TICKS(20000)) == pdTRUE) {
                    if ((msg.header.cmd == ADD) && (calc_crc16_msg((uint8_t *)&msg, sizeof(msg) - sizeof(uint16_t)) == msg.crc)) {
                        ESP_LOGI(TAG_MAIN, "Receive ADD command from gateway");

                        set_status_registration(NORMAL_MODE_DOOR_SENSOR);

                        /* Set time for internal rtc */
                        memcpy(&target_time, msg.payload, sizeof(time_t));
                        ESP_LOGI(TAG_MAIN, "Target time value: %lld", target_time);
                        set_rtc_time(target_time);
                        toggle_led(3, 500);

                        time_sleep = enter_deep_sleep_until_slot(get_device_id());
                        if (time_sleep > 0) {
                            /* Enter in deep sleep mode */
                            enter_in_deep_sleep_mode(time_sleep);
                        }
                    } else {
                        ESP_LOGW(TAG_MAIN, "Warning, command or crc16 not valid, discard message");
                    }
                }
            }
        break;
        /* Normal mode */
        case 2:
            ESP_LOGI(TAG_MAIN, "Normal mode");
        break;
        /* Mode unknown */
        default:
            ESP_LOGI(TAG_MAIN, "Mode unknown");
            esp_restart();
        break;
    }

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG_MAIN, "Wakeup from timer");
        break;
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG_MAIN, "Wakeup from GPIO %u", GPIO_WAKEUP_PIN);

            time_sleep = enter_deep_sleep_until_slot(get_device_id());
            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            status_node sdr = {
                .battery_low_detect = check_status_battery(),
                .state = new_state,
            };
            send_message(dst_mac, build_node_msg(UPDATE, get_device_id(), SENSOR, (esp_random() % 256), src_mac, get_device_name(), &sdr));

            /* Enter in deep sleep mode */
            if(time_sleep > 0) {
                enter_in_deep_sleep_mode(time_sleep);
            }
        break;
        default:
            ESP_LOGW(TAG_MAIN, "Warning, source wakeup unknown. May be first boot");
        break;
    }

    /* Slot time of 10 seconds */
    time_t awake_time = calculate_awake_time_in_slot(get_device_id());

    time(&t_start);
    t_end = t_start;
    while ((t_end - t_start) < awake_time) {

        /* Receive message from gateway */
        memset(&msg, 0, sizeof(msg));
        if (xQueueReceive(node_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG_MAIN, "Receive message from callback function");

            if (calc_crc16_msg((uint8_t *)&msg, sizeof(msg) - sizeof(uint16_t)) != msg.crc) {
                ESP_LOGW(TAG_MAIN, "Warning, crc16 not correct discard message");
            } else {

                gpio_debounce_filter(GPIO_WAKEUP_PIN);
                status_node sdr = {
                    .battery_low_detect = check_status_battery(),
                    .state = new_state,
                };

                switch (msg.header.cmd) {
                    case SYNC:
                        ESP_LOGI(TAG_MAIN, "Receive SYNC command from gateway");

                        memcpy(&target_time, (time_t *)msg.payload, sizeof(time_t));
                        set_rtc_time(target_time);
                    break;
                    case UPDATE:
                        ESP_LOGI(TAG_MAIN, "Receive UPDATE command from gateway");
                    break;
                    default:
                        ESP_LOGE(TAG_MAIN, "Command not found");
                    break;
                }

                msg = build_node_msg(msg.header.cmd, get_device_id(), SENSOR, msg.header.id_msg, src_mac, get_device_name(), &sdr);
                if(!send_message(dst_mac, msg)) {
                    ESP_LOGW(TAG_MAIN, "Warning, message not sent");
                }
            }
        }

        time(&t_end);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    /* Enter in deep sleep mode */
    enter_in_deep_sleep_mode(time_sleep);

    return;
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

/* Main program */
void app_main() {

    esp_err_t err = ESP_FAIL;

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

    node_queue = xQueueCreate(NODE_QUEUE_SIZE, sizeof(node_msg_t));
    if(!node_queue) {
        ESP_LOGE(TAG_MAIN, "Error, node queue not allocated");
        return;
    }

    /* Start configuration or normal mode */
    if (check_usb_connection()) {
        if(xTaskCreate(configuration_task, "configuration_task", 1024 * 3, NULL, 1, NULL) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Error, configuration task not started");
            return;
        }
    } else  {
        if(xTaskCreate(normal_mode_task, "normal_mode_task", 1024 * 3, NULL, 1, NULL) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Error, normal mode task not started");
            return;
        }
    }

    return;
}

#if 0
void app_main() {

    time_t t_start = 0;
    time_t t_end = 0;

    time(&t_start);
    t_end = t_start;
    while(t_end - t_start < 10) {
        ESP_LOGI(TAG_MAIN, "T start: %lld", t_start);
        time(&t_end);
        ESP_LOGI(TAG_MAIN, "ESP get time: %lld", esp_timer_get_time());
        ESP_LOGI(TAG_MAIN, "Time: %lld", t_end - t_start);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    enter_in_deep_sleep_mode(2);

    return;
}
#endif