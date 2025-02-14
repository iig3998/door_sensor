#include <stdio.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
//#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "web_server.h"

#define WIFI_SSID            "DomoticHouse"
#define WIFI_PASSWORD        "D0m0t1cH0use"
#define DOOR_SENSOR_NAME_LEN 15
#define TAG_WEBSERVER        "WEB_SERVER"
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

RTC_DATA_ATTR uint8_t device_id = 0;
RTC_DATA_ATTR char device_name[DOOR_SENSOR_NAME_LEN];
RTC_DATA_ATTR bool configured = false;

static EventGroupHandle_t wifi_event_group;

const char html_page[] = R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configurazione Dispositivo</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 50px;
            background: linear-gradient(to right, #4facfe, #00f2fe);
            color: white;
        }
        .container {
            display: inline-block;
            text-align: left;
            background: rgba(255, 255, 255, 0.2);
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        label, input, button {
            display: block;
            margin: 10px 0;
        }
        input {
            padding: 8px;
            border: none;
            border-radius: 5px;
        }
        input[type=number] {
            -moz-appearance: textfield;
        }
        input[type=number]::-webkit-outer-spin-button,
        input[type=number]::-webkit-inner-spin-button {
            -webkit-appearance: none;
            margin: 0;
        }
        button {
            padding: 10px 15px;
            border: none;
            border-radius: 5px;
            background: #ff7eb3;
            color: white;
            cursor: pointer;
            transition: background 0.3s;
        }
        button:hover {
            background: #ff4f8b;
        }
    </style>
</head>
<body>
    <h1>Configurazione Dispositivo</h1>
    <div class="container">
        <label for="number">Numero (0-255):</label>
        <input type="number" id="number" min="0" max="255" required>
        <label for="name">Nome dispositivo (max 15 caratteri):</label>
        <input type="text" id="name" maxlength="15" required>
        <button onclick="salvaDati()">Salva</button>
        <button onclick="cancellaDati()">Cancella</button>
    </div>
    <script>
        function salvaDati() {
            let number = document.getElementById("number").value;
            let name = document.getElementById("name").value;
            if (number === "" || device_name === "") {
                alert("Compila tutti i campi!");
                return;
            }
            fetch('/save?number=' + number + '&name=' + encodeURIComponent(name))
                .then(response => response.text())
                .then(data => alert(data));
        }

        function cancellaDati() {
            document.getElementById("number").value = "";
            document.getElementById("name").value = "";
        }
    </script>
</body>
</html>
)rawliteral";

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

    uint8_t retry_num = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < 3) {
            esp_wifi_connect();
            retry_num ++;
            ESP_LOGW(TAG_WEBSERVER, "Warning, retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_WEBSERVER,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG_WEBSERVER, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }

    return;
}

/* Get id device */
uint8_t get_id_device() {

    return device_id;
}

/* Get name device */
char *get_name_device() {
    
    return device_name;
}

/**/
bool is_device_configured() {

    return configured;
}

/* Home page handler */
esp_err_t home_page_handler(httpd_req_t *req) {

    ESP_LOGI(TAG_WEBSERVER, "Send home page");

    httpd_resp_set_type(req, "text/html");

    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* Save data handler */
esp_err_t save_data_handler(httpd_req_t *req) {

    char param[32];

    ESP_LOGI(TAG_WEBSERVER, "Save data");

    memset(param, '\0', sizeof(param));

    if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
        
        char number[4];
        char name[16];

        memset(number, '\0', sizeof(number));
        memset(name, '\0', sizeof(name));

        if (httpd_query_key_value(param, "number", number, sizeof(number)) == ESP_OK && httpd_query_key_value(param, "name", name, sizeof(name)) == ESP_OK) {

            device_id = atoi(number);
            strncpy(device_name, name, sizeof(device_name));

            ESP_LOGI(TAG_WEBSERVER, "Data saved: ID number: %d, Name sensor: %s", device_id, device_name);

            httpd_resp_send(req, "Data saved successfully!", HTTPD_RESP_USE_STRLEN);
            configured = true;

            return ESP_OK;
        }
    }
    
    httpd_resp_send_500(req);

    return ESP_FAIL;
}

static httpd_uri_t save_data = {
    .uri = "/save",
    .method = HTTP_GET,
    .handler = save_data_handler,
    .user_ctx = NULL
};

static httpd_uri_t home_page = {
    .uri       = "/home",
    .method    = HTTP_GET,
    .handler   = home_page_handler,
    .user_ctx  = NULL
};

/* Init WiFi station for webserver */
esp_err_t wifi_init_sta_ws() {

    esp_err_t err = ESP_FAIL;
    EventBits_t bits;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    wifi_event_group = xEventGroupCreate();

    err = esp_netif_init();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, network interface not init");
        return err;
    }

    err = esp_event_loop_create_default();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, event loop not created");
        return err;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, WiFi not inited");
        return err;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    /* Set WiFi mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, WiFi mode not setted");
        return err;
    }

    /* COnfigure WiFi */
    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, WiFi not configured");
        return err;
    }

    /* WiFi start */
    err = esp_wifi_start();
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, WiFi not started");
        return err;
    }

    bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WEBSERVER, "Connected to AP SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WEBSERVER, "Failed to connect to SSID: %s", WIFI_SSID);
    }

    return err;
}

/* Start webserver */
httpd_handle_t start_webserver() {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {

        httpd_register_uri_handler(server, &home_page);
        httpd_register_uri_handler(server, &save_data);
    } else {
        ESP_LOGE(TAG_WEBSERVER, "Error, web server door sensor not started");
        return NULL;
    }

    return server;
}

/* Stop webserver */
void stop_webserver(httpd_handle_t server) {

    esp_err_t err = ESP_FAIL;

    err = httpd_stop(server);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, webserver not stopped");
        return;
    }

    esp_wifi_stop();

    esp_wifi_deinit();

    return;
}