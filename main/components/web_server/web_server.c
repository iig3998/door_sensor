#include <stdio.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "mdns.h"

#include "nvs_mgmt.h"
#include "web_server.h"

#define TAG_WEBSERVER        "WEB_SERVER"
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1
#define DEVICE_NUMBER_SIZE   4
#define DEVICE_NAME_SIZE     16

static httpd_handle_t server = NULL;
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
            margin: 20px;
            background: linear-gradient(to right, #6dd5ed, #2193b0);
            color: black;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            position: relative;
            overflow: hidden;
        }
        .container {
            display: inline-block;
            text-align: left;
            background: #ffffff;
            padding: 25px;
            border-radius: 12px;
            box-shadow: 0 6px 10px rgba(0, 0, 0, 0.3);
            color: black;
            max-width: 400px;
        }
        label {
            font-weight: bold;
            color: #333;
            display: block;
            margin-top: 10px;
        }
        input {
            padding: 10px;
            border: 2px solid #2193b0;
            border-radius: 6px;
            background: #e0f7fa;
            width: 95%;
            color: #333;
            font-size: 14px;
            display: block;
            margin-bottom: 10px;
        }
        input[type=number] {
            -moz-appearance: textfield;
            width: 50%;
        }
        input[type=number]::-webkit-outer-spin-button,
        input[type=number]::-webkit-inner-spin-button {
            -webkit-appearance: none;
            margin: 0;
        }
        button {
            padding: 12px 18px;
            border: none;
            border-radius: 6px;
            background: #ff9800;
            color: white;
            cursor: pointer;
            transition: background 0.3s, transform 0.2s;
            font-weight: bold;
            margin-top: 15px;
            width: 100%;
        }
        button:hover {
            background: #f57c00;
            transform: scale(1.05);
        }
        .footer {
            width: auto;
            text-align: right;
            padding: 10px;
            position: fixed;
            bottom: 10px;
            right: 10px;
            font-size: 14px;
            font-weight: bold;
            color: black;
            background: rgba(255, 255, 255, 0.9);
            padding: 8px 12px;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0, 0, 0, 0.3);
        }
    </style>
</head>
<body>
    <h1>Configurazione sensore porta</h1>
    <div class="container">
        <label for="number">Numero (1-255):</label>
        <input type="number" id="device_id" min="1" max="255" required>
        <label for="name">Nome dispositivo (max 15 caratteri):</label>
        <input type="text" id="device_name" maxlength="15" required>
        <button onclick="salvaDati()">Salva</button>
        <button onclick="delete_configuration()" style="background: #d32f2f;">Cancella Configurazione</button>
        <button onclick="cancellaDati()">Reset</button>
    </div>

    <div class="footer">DomoticHouse v0.1.0 by Fabio Molon</div>

    <script>
        function salvaDati() {
            let device_id = document.getElementById("device_id").value;
            let device_name = document.getElementById("device_name").value.trim().replace(/\s+/g, "_");
            if (device_id === "" || device_name === "") {
                alert("Compila tutti i campi!");
                return;
            }
            fetch('/save?number=' + device_id + '&name=' + encodeURIComponent(device_name))
                .then(response => response.text())
                .then(data => alert(data));
        }

        function cancellaDati() {
            document.getElementById("device_id").value = "";
            document.getElementById("device_name").value = "";
        }

        function delete_configuration() {
            fetch('/delete')
                .then(response => response.text())
                .then(data => {
                    alert("Configurazione cancellata");
                    cancellaDati();
                });
        }
    </script>
</body>
</html>

)rawliteral";

/* WiFi event handler function */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

    uint8_t retry_num = 0;

    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (retry_num < 3) {
                esp_wifi_connect();
                retry_num ++;
                ESP_LOGW(TAG_WEBSERVER, "Warning, retry to connect to the AP %s", CONFIG_WIFI_SSID);
            } else {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG_WEBSERVER,"Connect to the AP fail");
        } 
    } else if (event_base == IP_EVENT) {

        uint8_t esp_mac[6];
        char hostname[32];
        memset(hostname, '\0', 32);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG_WEBSERVER, "Got IPv4 address: " IPSTR, IP2STR(&event->ip_info.ip));

        esp_read_mac(esp_mac, ESP_MAC_WIFI_STA);

        snprintf(hostname, 32, "%s-%u%u%u%u%u%u", (const char*)"door_sensor", esp_mac[0], esp_mac[1], esp_mac[2], esp_mac[3], esp_mac[4], esp_mac[5]);
        ESP_LOGI(TAG_WEBSERVER, "Hostname: %s", hostname);

        /* Init and configure the mDNS server */
        mdns_init();
        mdns_hostname_set(hostname);
        mdns_instance_name_set(hostname);

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }

    return;
}

/* Get device id */
uint8_t get_device_id() {

    esp_err_t err = ESP_FAIL;
    uint8_t value;

    err = read_uint8_from_nvs("storage", "device_id", &value);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, device id not read");
    }

    return value;
}

/* Get device name */
void get_device_name(char *device_name, uint8_t len) {

    esp_err_t err = ESP_FAIL;

    if(!device_name){
        ESP_LOGE(TAG_WEBSERVER, "Error, destination pointer for device name is null");
        return;
    }

    err = read_string_from_nvs("storage", "device_name", device_name, len);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, device name not read");
    }
    
    return;
}

/* Check if device is configured */
uint8_t is_device_configured() {

    esp_err_t err = ESP_FAIL;
    uint8_t value = 0;

    err = read_uint8_from_nvs("storage", "config", &value);
    if(err != ESP_OK) {
        ESP_LOGW(TAG_WEBSERVER, "Warning, variable config not present. Set config to 0");
        /* First configuration */
        save_uint8_to_nvs("storage", "config", value);
    }

    return value;
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
        
        char device_number[DEVICE_NUMBER_SIZE];
        char device_name[DEVICE_NAME_SIZE];

        memset(device_number, '\0', sizeof(device_number));
        memset(device_name, '\0', sizeof(device_name));

        ESP_LOGI(TAG_WEBSERVER, "Param: %s", param);
        if (!httpd_query_key_value(param, "number", device_number, sizeof(device_number)) && !httpd_query_key_value(param, "name", device_name, sizeof(device_name))) {

            ESP_LOGI(TAG_WEBSERVER, "Data saved: ID number: %s, Name sensor: %s", device_number, device_name);

            /* Save variable in storage */
            save_uint8_to_nvs("storage", "config", 1);
            save_uint8_to_nvs("storage", "device_id", atoi(device_number));
            save_string_to_nvs("storage", "device_name", device_name);

            httpd_resp_send(req, "Data saved successfully!", HTTPD_RESP_USE_STRLEN);

            return ESP_OK;
        }
    }
    
    httpd_resp_send_500(req);

    return ESP_FAIL;
}

/* Delete data handler */
esp_err_t delete_data_handler(httpd_req_t *req) {

    ESP_LOGI(TAG_WEBSERVER, "Delete data");

    /* Delete variable in storage */
    save_uint8_to_nvs("storage", "config", 0);
    save_uint8_to_nvs("storage", "device_id", 0);
    save_string_to_nvs("storage", "device_name", '\0');

    httpd_resp_send(req, "Data delete successfully!", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;

}

static httpd_uri_t home_page = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = home_page_handler,
    .user_ctx  = NULL
};

static httpd_uri_t save_data = {
    .uri = "/save",
    .method = HTTP_GET,
    .handler = save_data_handler,
    .user_ctx = NULL
};

static httpd_uri_t delete_data = {
    .uri = "/delete",
    .method = HTTP_DELETE,
    .handler = delete_data_handler,
    .user_ctx = NULL
};

/* Init WiFi station for webserver */
esp_err_t init_webserver() {

    esp_err_t err = ESP_FAIL;
    EventBits_t bits;
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    wifi_event_group = xEventGroupCreate();

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
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    /* Set WiFi mode */
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, WiFi mode not setted");
        return err;
    }

    /* Configure WiFi */
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
        ESP_LOGI(TAG_WEBSERVER, "Connected to AP SSID: %s", CONFIG_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WEBSERVER, "Failed to connect to SSID: %s", CONFIG_WIFI_SSID);
    }

    return err;
}

/* Start webserver */
esp_err_t start_webserver() {

    esp_err_t err = ESP_FAIL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, web server door sensor not started");
        return err;
    }

    httpd_register_uri_handler(server, &home_page);
    httpd_register_uri_handler(server, &save_data);
    httpd_register_uri_handler(server, &delete_data);

    return err;
}

/* Stop webserver */
esp_err_t stop_webserver() {

    esp_err_t err = ESP_FAIL;

    err = httpd_stop(server);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, webserver not stopped");
        return err;
    }

    /* Stop wifi */
    esp_wifi_stop();

    /* Deinit wifi */
    esp_wifi_deinit();

    return err;
}