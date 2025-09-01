#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_console.h"

#include "wifi.h"
#include "adc.h"
#include "node.h"
#include "common.h"
#include "conf.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#define DEBOUNCE_COUNTER      50
#define NUMBER_ATTEMPTS       3
#define ESPNOW_WIFI_CHANNEL   11
#define RETRASMISSION_TIME_MS 50

#define DATA_SENT_SUCCESS     (1 << 0)
#define DATA_SENT_FAILED      (1 << 1)
#define DATA_RECEIVED_SUCCESS (1 << 2)
#define DATA_RECEIVED_FAILED  (1 << 3)

#define NODE_QUEUE_SIZE       4

#define GPIO_WAKEUP_PIN       GPIO_NUM_25
#define LED_ON_BOARD          GPIO_NUM_5

#define TAG_MAIN              "MAIN"

RTC_DATA_ATTR bool new_state = 0;
RTC_DATA_ATTR bool old_state = 0;

/* MAC address gateway */
static uint8_t dst_mac[MAC_SIZE] = {0x78, 0x42, 0x1C, 0x6A, 0xEF, 0x94};
static uint8_t src_mac[MAC_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static QueueHandle_t node_queue;
static EventGroupHandle_t xEventGroupDoorSensor;

/* Receive callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    ESP_LOGI(TAG_MAIN, "Receive callback function");

    if (!recv_info->src_addr || !data || len <= 0) {
        return;
    }

    node_msg_t msg;
    memcpy(&msg, data, sizeof(node_msg_t));

    if (xQueueSend(node_queue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
        ESP_LOGW(TAG_MAIN, "Warning, queue is full, discard message");
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

/* Send message */
static bool send_message(uint8_t dst_mac[], node_msg_t msg) {

    esp_err_t err = ESP_FAIL;
    EventBits_t uxBits;

    for (uint8_t i = 0; i < NUMBER_ATTEMPTS; i++) {

        xEventGroupClearBits(xEventGroupDoorSensor, DATA_SENT_SUCCESS | DATA_SENT_FAILED);

        err = esp_now_send(dst_mac, (uint8_t *)&msg, sizeof(msg));
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Waits callback function result */
        uxBits = xEventGroupWaitBits(
            xEventGroupDoorSensor,
            DATA_SENT_SUCCESS | DATA_SENT_FAILED,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(RETRASMISSION_TIME_MS)
        );

        /* Success */
        if (uxBits & DATA_SENT_SUCCESS) {
            return true;
        }

        /* Failed */
        if (uxBits & DATA_SENT_FAILED) {
            ESP_LOGW(TAG_MAIN, "Send failed, retrying... (%u/%u)", i + 1, NUMBER_ATTEMPTS);
        } else {
            ESP_LOGW(TAG_MAIN, "Timeout waiting for send result, retrying... (%u/%u)", i + 1, NUMBER_ATTEMPTS);
        }
    }

    return false;
}

/* Toggle led */
static void toggle_led(const uint16_t num_flash, const uint16_t time_flash) {

    for(uint16_t i = 0; i < num_flash; i++) {
        gpio_set_level(LED_ON_BOARD, 0);
        vTaskDelay(pdMS_TO_TICKS(time_flash));
        gpio_set_level(LED_ON_BOARD, 1);
        vTaskDelay(pdMS_TO_TICKS(time_flash));
    }

    return;
}

/* Set command */
static int cmd_set(int argc, char **argv) {

    if(argc != 3) {
        printf("Uso: set device_name | device_id <name> | <id>\r\n");
        return -1;
    }

    if(!strncmp(argv[1], "device_name", strlen("device_name"))) {
        printf("Imposta nome dispositivo\r\n");
        if (strlen(argv[2]) > 0 && strlen(argv[2]) <= 15) {
            set_device_name(argv[2]);
        } else {
            printf("Nome dispositivo non valido\r\n");
        }

    } else if(!strncmp(argv[1], "device_id", strlen("device_id"))) {
        printf("Imposta id dispositivo\n");
        if (atoi(argv[2]) > 0 && atoi(argv[2]) <= 10) {
            set_device_id(atoi(argv[2]));
        } else {
            printf("Valore id non valido\r\n");
        }
    } else {
        printf("Argomento non valido: %s\r\n", argv[1]);
        return -1;
    }

    return 0;
}

/* Del command */
static int cmd_del(int argc, char **argv) {

    if (argc != 2) {
        printf("Uso: del name | id\r\n");
        return -1;
    }

    if(get_status_registration() == UNREGISTRATION_DOOR_SENSOR) {
        if (!strncmp(argv[1], "device_name", strlen("device_name"))) {
            printf("Cancella il nome del dispositivo\r\n");
            del_device_name();
        } else if (!strncmp(argv[1], "device_id", strlen("device_id"))) {
            printf("Cancella id dispositivo\r\n");
            del_device_id();
        } else {
            printf("Argomento non valido: %s\r\n", argv[1]);
            return -1;
        }
    } else {
        printf("Attenzione, prima di cancellare la configurazione del dispositvo deregistrarlo dalla centrale\r\n");
        return -1;
    }

    return 0;
}

/* Get command */
static int cmd_get(int argc, char **argv) {

    if (argc != 2) {
        printf("Uso: get device_name | device_id | mac\r\n");
        return -1;
    }

    if (!strncmp(argv[1], "device_name", strlen("device_name"))) {
        char *name = get_device_name();
        if(name)
            printf("Nome dispsitivo: %s\r\n", name);
        else
            printf("Nome dispositivo non impostato\r\n");

    } else if (!strncmp(argv[1], "device_id", strlen("device_id"))) {
        uint8_t id = get_device_id();
        if(id)
            printf("Id dispisitivo: %u\r\n", get_device_id());
        else
            printf("Id del dispositivo non impostato\r\n");

    } else if (!strncmp(argv[1], "device_mac", strlen("mac"))) {
        printf("MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    }

    return 0;
}

/* Registration/Unregistratioon command */
static int cmd_register(int argc, char **argv) {

    if (argc != 2) {
        printf("Uso: register | deregister device\r\n");
        return -1;
    }

    const char *cmd = argv[0];
    cmd_type action;

    if (!strncmp(cmd, "register", strlen("register")))
        action = ADD;
    else if (!strncmp(cmd, "deregister", strlen("deregister")))
        action = DEL;
    else {
        printf("Comando non valido.\r\n");
        return -1;
    }

    if(get_device_name() && get_device_id()) {

        node_msg_t msg_sent = build_node_msg(
            action,
            get_device_id(),
            SENSOR,
            esp_random() % 256,
            src_mac,
            get_device_name(),
            NULL
        );

        if (!send_message(dst_mac, msg_sent)) {
            printf("%s non avvenuta. Assicurarsi che la centralina sia accesa e raggiungibile.\r\n", action == ADD ? "Registrazione" : "Deregistrazione");
            return -1;
        } else {
            node_msg_t msg_received;
            if(xQueueReceive(node_queue, &msg_received, pdMS_TO_TICKS(2000)) == pdTRUE) {
                if ((msg_received.header.cmd == action) && (calc_crc16_msg((uint8_t *)&msg_received, sizeof(msg_received) - sizeof(uint16_t)) == msg_received.crc) && (msg_sent.header.id_msg == msg_received.header.id_msg)) {
                    ESP_LOGI(TAG_MAIN, "Receive %s command from gateway", action == ADD ? "add" : "del");

                    if(action == ADD) {
                        set_status_registration(REGISTRATION_DOOR_SENSOR);
                    } else if(action == DEL) {
                        set_status_registration(UNREGISTRATION_DOOR_SENSOR);
                    }

                    toggle_led(3, 500);

                } else {
                    printf("Attenzione, il messaggio ricevuto dalla centralina è corrotto. Riprovare.\r\n");
                    return -1;
                }
            } else {
                printf("%s non avvenuta. Ricontrollare la configurazione del dispositivo.\r\n", action == ADD ? "Registrazione" : "Deregistrazione");
                return -1;
            }
        }
        
    } else {
        printf("Nome del dispositivo o identificativo non impostato.\r\n");
        return -1;
    }

    return 0;
}

/* List command available for manage device */
static void register_commands(void) {

    const esp_console_cmd_t set_cmd = {
        .command = "set",
        .help = "Imposta il nome del dispositivo o il suo identificativo",
        .hint = NULL,
        .func = &cmd_set,
    };

    esp_console_cmd_register(&set_cmd);

    const esp_console_cmd_t del_cmd = {
        .command = "del",
        .help = "Cancella il nome del dispositivo o il suo identificativo",
        .hint = NULL,
        .func = &cmd_del,
    };

    esp_console_cmd_register(&del_cmd);

    const esp_console_cmd_t get_cmd = {
        .command = "get",
        .help = "Ritorna la configurazione del dispositivo",
        .hint = NULL,
        .func = &cmd_get,
    };

    esp_console_cmd_register(&get_cmd);

    const esp_console_cmd_t registration_cmd = {
        .command = "register",
        .help = "Registra il dispositivo alla centrale",
        .hint = NULL,
        .func = &cmd_register,
    };

    esp_console_cmd_register(&registration_cmd);

    const esp_console_cmd_t deregistration_cmd = {
        .command = "deregister",
        .help = "Deregistra il dispositivo dalla centrale",
        .hint = NULL,
        .func = &cmd_register,
    };

    esp_console_cmd_register(&deregistration_cmd);

    return;

}

/* Init console */
static int8_t init_console() {

    esp_err_t err = ESP_FAIL;
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    repl_config.prompt = "domotichouse$";
    repl_config.max_cmdline_length = 100;

    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    err = esp_console_new_repl_uart(&hw_config, &repl_config, &repl);
    if(err != ESP_OK)
        return -1;

    esp_console_register_help_command();

    err = esp_console_start_repl(repl);
    if(err != ESP_OK)
        return -1;

    register_commands();

    return 0;
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
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return;
}

/* Enter in deep sleep mode */
static void enter_in_deep_sleep_mode(uint16_t time_sleep) {

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

    vTaskDelay(pdMS_TO_TICKS(20));

    esp_deep_sleep_start();
}

/* Init trasmission */
esp_err_t init_transmission() {

    esp_err_t err = ESP_FAIL;
    esp_now_peer_info_t peer;

    /* Init WiFi station */
    err = init_wifi_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, WiFi not configurated. Restart device");
        return err;
    }

    /* Init espnow */
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, espnow not inited");
        return err;
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
        return err;
    }

    /* Register send callback function */
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered. Restart device.");
        return err;
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, send callback function not registered. Restart device.");
        return err;
    }

    return err;
}

#define MIN_SLEEP_MIN 10
#define MAX_SLEEP_MIN 13

/* Normal mode task */
static void normal_mode_task(void *arg) {

    node_msg_t msg_sent;
    node_msg_t msg_received;
    status_node sdr;

    memset(&msg_sent, 0, sizeof(msg_sent));
    memset(&msg_received, 0, sizeof(msg_received));
    memset(&sdr, 0, sizeof(sdr));

    switch(get_status_registration()) {
        /* Unregistration mode */
        case UNREGISTRATION_DOOR_SENSOR:
            ESP_LOGI(TAG_MAIN, "Unregistration mode. Please registrate device.");
            while(1) {
                gpio_set_level(LED_ON_BOARD, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_ON_BOARD, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
            }            
        break;
        /* Registration mode */
        case REGISTRATION_DOOR_SENSOR:
            ESP_LOGI(TAG_MAIN, "Registration mode");

            gpio_debounce_filter(GPIO_WAKEUP_PIN);
            sdr.battery_low_detect = check_status_battery();
            sdr.state = new_state;

            esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
            switch (wakeup_reason) {
                case ESP_SLEEP_WAKEUP_TIMER:
                    ESP_LOGI(TAG_MAIN, "Wakeup from timer");
                    msg_sent = build_node_msg(UPDATE, get_device_id(), SENSOR, (esp_random() % 256), src_mac, get_device_name(), &sdr);
                break;
                case ESP_SLEEP_WAKEUP_GPIO:
                    ESP_LOGI(TAG_MAIN, "Wakeup from GPIO %u", GPIO_WAKEUP_PIN);
                    msg_sent = build_node_msg(ACTIVE_ALARM, get_device_id(), SENSOR, (esp_random() % 256), src_mac, get_device_name(), &sdr);
                break;
                default:
                    ESP_LOGW(TAG_MAIN, "Warning, source wakeup unknown. May be first boot");
                    msg_sent = build_node_msg(UPDATE, get_device_id(), SENSOR, (esp_random() % 256), src_mac, get_device_name(), &sdr);
                break;
            }
        break;
    }

    /* Temporal window */
    for(uint8_t retry = 0; retry < 3; retry ++) {

        /* Send message */
        if(send_message(dst_mac, msg_sent)) {
            if(xQueueReceive(node_queue, &msg_received, pdMS_TO_TICKS(2000)) == pdTRUE) {
                if ((msg_received.header.cmd == UPDATE) && (calc_crc16_msg((uint8_t *)&msg_received, sizeof(msg_received) - sizeof(uint16_t)) == msg_received.crc) && (msg_sent.header.id_msg == msg_received.header.id_msg)) {
                    ESP_LOGI(TAG_MAIN, "Receive response from gateway. Exit");
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    uint32_t sleep_sec = (MIN_SLEEP_MIN * 60) + ((esp_random() + get_device_id()) % ((MAX_SLEEP_MIN - MIN_SLEEP_MIN + 1) * 60));
    
    ESP_LOGI(TAG_MAIN, "Sensor sleeping for %lu seconds", sleep_sec);

    /* Enter in deep sleep mode */
    enter_in_deep_sleep_mode(sleep_sec);

    return;
}

/* Main program */
void app_main(void) {

    esp_err_t err = ESP_FAIL;

    init_conf();

    /* Init network interface */
    err = esp_netif_init();
    if (err != ESP_OK) {
        esp_restart();
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        esp_restart();
    }

    xEventGroupDoorSensor = xEventGroupCreate();
    if (!xEventGroupDoorSensor) {
        ESP_LOGE(TAG_MAIN, "Error, event group not created. Restart device");
        esp_restart();
    }

    node_queue = xQueueCreate(NODE_QUEUE_SIZE, sizeof(node_msg_t));
    if(!node_queue) {
        ESP_LOGE(TAG_MAIN, "Error, node queue not allocated");
        esp_restart();
    }

    err = init_transmission();
    if (err != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    /* Start cli or normal mode */
    if (!check_usb_connection()) {
        init_console();
    } else  {
        if(xTaskCreate(normal_mode_task, "normal_mode_task", 1024 * 2, NULL, 1, NULL) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Error, normal mode task not started. Restart device");
            esp_restart();
        }
    }

    return;
}

/* Pre app main program */
__attribute__((constructor)) void pre_app_main() {

    esp_err_t err = ESP_FAIL;

    /* Suppress boot messages */
    esp_deep_sleep_disable_rom_logging();

    /* Read MAC address */
    err = esp_read_mac(src_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_MAIN, "Error, MAC address not read. Restart device");
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