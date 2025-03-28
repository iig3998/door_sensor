#include <stdio.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_mgmt.h"
#include "web_server.h"


#define WIFI_SSID            "DOOR_SENSOR"
#define WIFI_PASS            "12345678"
#define MAX_STA_CONN         1
#define TAG_WEBSERVER        "WEB_SERVER"
#define DEVICE_NUMBER_SIZE   4
#define DEVICE_NAME_SIZE     16
#define BUFFER_PARAMETR_SIZE 32
#define HOSTNAME_SIZE        32

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

    </script>
</body>
</html>

)rawliteral";


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

/* Get configuration */
bool get_conf() {

    uint8_t state_conf = 0;
    
    read_uint8_from_nvs("storage", "config", &state_conf);

    if(state_conf)
        return true;

    return false;
}

/* Home page handler */
esp_err_t home_page_handler(httpd_req_t *req) {

    ESP_LOGI(TAG_WEBSERVER, "Home page");

    httpd_resp_set_type(req, "text/html");

    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* Save data handler */
esp_err_t save_data_handler(httpd_req_t *req) {

    char buffer_param[BUFFER_PARAMETR_SIZE];

    ESP_LOGI(TAG_WEBSERVER, "Save data");

    memset(buffer_param, '\0', sizeof(buffer_param));

    if (httpd_req_get_url_query_str(req, buffer_param, sizeof(buffer_param)) == ESP_OK) {
        
        char device_number[DEVICE_NUMBER_SIZE];
        char device_name[DEVICE_NAME_SIZE];

        memset(device_number, '\0', sizeof(device_number));
        memset(device_name, '\0', sizeof(device_name));

        if (!httpd_query_key_value(buffer_param, "number", device_number, sizeof(device_number)) && !httpd_query_key_value(buffer_param, "name", device_name, sizeof(device_name))) {

            ESP_LOGI(TAG_WEBSERVER, "Data saved: ID number: %s, Name sensor: %s", device_number, device_name);

            /* Reset configuration */
            save_uint8_to_nvs("storage", "config", 0);
            save_uint8_to_nvs("storage", "device_id", 0);
            save_string_to_nvs("storage", "device_name", "");

            /* Save variable in storage */
            save_uint8_to_nvs("storage", "config", 1);
            save_uint8_to_nvs("storage", "device_id", atoi(device_number));
            save_string_to_nvs("storage", "device_name", device_name);

            httpd_resp_send(req, "Data saved successfully! Remove USB cable and restart device.", HTTPD_RESP_USE_STRLEN);

            return ESP_OK;
        }
    }
    
    httpd_resp_send_500(req);

    return ESP_FAIL;
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

/* Configure access point esp32 ddor sensor */
void wifi_init_softap() {

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_AP);
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_OPEN
        },
    };
    
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    
    esp_wifi_start();

    ESP_LOGI(TAG_WEBSERVER, "Access Point run. SSID:%s Password:%s", WIFI_SSID, WIFI_PASS);

    return;
}

/* Start webserver */
esp_err_t start_webserver(httpd_handle_t server) {
    
    esp_err_t err = ESP_FAIL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    err = httpd_start(&server, &config);
    if (err == ESP_OK) {
        httpd_register_uri_handler(server, &home_page);
        httpd_register_uri_handler(server, &save_data);
    } else {
        ESP_LOGE(TAG_WEBSERVER, "Error web server not started");
        return err;
    }
    
    return err;
}

/* Stop webserver */
void stop_webserver(httpd_handle_t server) {

    if (server) {
        httpd_stop(server);
        server = NULL;
    }

    return;
}