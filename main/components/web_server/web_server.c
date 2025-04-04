#include <stdio.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_mgmt.h"
#include "web_server.h"

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
            }
            .container {
                text-align: left;
                background: #ffffff;
                padding: 30px;
                border-radius: 12px;
                box-shadow: 0 6px 10px rgba(0, 0, 0, 0.3);
                color: black;
                max-width: 472px;
            }
            label {
                font-weight: bold;
                color: #333;
                display: block;
                margin-top: 10px;
            }
        input {
            padding: 12px;
            border: 2px solid #2193b0;
            border-radius: 6px;
            background: #e0f7fa;
            width: calc(100% - 24px);
            color: #333;
            font-size: 16px;
            display: block;
            margin-bottom: 12px;
        }
        .button-container {
            display: flex;
            justify-content: space-between;
            margin-top: 45px;
        }
        button {
            padding: 14px 20px;
            border: none;
            border-radius: 6px;
            background: #ff9800;
            color: white;
            cursor: pointer;
            transition: background 0.3s, transform 0.2s;
            font-weight: bold;
            width: 30%;
            text-align: center;
            white-space: nowrap;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        button:hover {
            background: #f57c00;
            transform: scale(1.05);
        }
        .footer {
            text-align: right;
            padding: 10px;
            position: fixed;
            bottom: 10px;
            right: 10px;
            font-size: 14px;
            font-weight: bold;
            background: rgba(255, 255, 255, 0.9);
            padding: 8px 12px;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0, 0, 0, 0.3);
        }
        #errore {
            color: red;
            font-weight: bold;
            margin-top: 10px;
        }
    </style>
</head>
<body>
    <h1>Configurazione sensore porta</h1>
    <div class="container">
        <label for="device_id">Numero (1-255):</label>
        <input type="number" id="device_id" min="1" max="255" required>
        <label for="device_name">Nome dispositivo (max 15 caratteri):</label>
        <input type="text" id="device_name" maxlength="15" required>
        <div id="errore"></div>
        <div class="button-container">
            <button onclick="salvaDati()">Configura</button>
            <button onclick="cancellaDati()">Reset</button>
            <button onclick="cancellaRegistrazione()">Cancella configurazione</button>
        </div>
    </div>

    <div class="footer">DomoticHouse v0.1.0 by Fabio Molon</div>

    <script>
        function salvaDati() {
            let device_id = document.getElementById("device_id").value.trim();
            let device_name = document.getElementById("device_name").value.trim().replace(/\s+/g, "_");
            let errore = document.getElementById("errore");
            errore.innerText = "";
            
            if (!device_id || !device_name) {
                errore.innerText = "Compila tutti i campi!";
                return;
            }

            let numero = parseInt(device_id);
            if (isNaN(numero) || numero < 1 || numero > 255) {
                errore.innerText = "Il numero deve essere tra 1 e 255.";
                return;
            }

            if (device_name.length > 15) {
                errore.innerText = "Il nome deve avere massimo 15 caratteri.";
                return;
            }

            fetch(`/save?number=${device_id}&name=${encodeURIComponent(device_name)}`,  {method: 'POST'})
                .then(response => response.text())
                .then(data => alert(data))
                .catch(() => errore.innerText = "Errore di connessione al server!");
        }

        function cancellaDati() {
            document.getElementById("device_id").value = "";
            document.getElementById("device_name").value = "";
            document.getElementById("errore").innerText = "";
        }

        function cancellaRegistrazione() {
            if (confirm("Sei sicuro di voler cancellare la registrazione del dispositivo?")) {
                fetch('/delete',  {method: 'DELETE'})
                    .then(response => response.text())
                    .then(data => alert(data))
                    .catch(() => alert("Errore nella cancellazione della registrazione!"));
                cancellaDati();
            }
        }
    </script>
    </body>
    </html>
)rawliteral";

/* Get device id */
uint8_t get_device_id() {

    esp_err_t err = ESP_FAIL;
    uint8_t device_id;

    err = read_uint8_from_nvs("storage", "device_id", &device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, device id not read");
        return 0;
    }

    return device_id;
}

/* Get device name */
void get_device_name(char *device_name, uint8_t len) {

    esp_err_t err = ESP_FAIL;

    if (!device_name) {
        ESP_LOGE(TAG_WEBSERVER, "Error, destination pointer for device name is null");
        return;
    }

    err = read_string_from_nvs("storage", "device_name", device_name, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, device name not read");
    }

    return;
}

uint8_t get_status_registered() {

    esp_err_t err = ESP_FAIL;
    uint8_t registered;

    err = read_uint8_from_nvs("storage", "registered", &registered);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, registered not read");
        return 0;
    }

    return registered;
}

void set_status_registered(uint8_t registered) {

    esp_err_t err = ESP_FAIL;
    
    err = read_uint8_from_nvs("storage", "registered", &registered);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, registered not read");
    }

    return;
}

/* Home page handler */
static esp_err_t home_page_handler(httpd_req_t *req) {

    ESP_LOGI(TAG_WEBSERVER, "Home page");

    httpd_resp_set_type(req, "text/html");

    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* Save data page handler */
static esp_err_t save_data_handler(httpd_req_t *req) {

    char buffer_param[BUFFER_PARAMETR_SIZE];

    ESP_LOGI(TAG_WEBSERVER, "Save data page handler");

    memset(buffer_param, '\0', sizeof(buffer_param));

    if (httpd_req_get_url_query_str(req, buffer_param, sizeof(buffer_param)) == ESP_OK) {

        char device_number[DEVICE_NUMBER_SIZE];
        char device_name[DEVICE_NAME_SIZE];

        memset(device_number, '\0', sizeof(device_number));
        memset(device_name, '\0', sizeof(device_name));

        if (!httpd_query_key_value(buffer_param, "number", device_number, sizeof(device_number)) && !httpd_query_key_value(buffer_param, "name", device_name, sizeof(device_name))) {

            ESP_LOGI(TAG_WEBSERVER, "ID number: %s, Name sensor: %s", device_number, device_name);

            /* Clear configuration in storage */
            save_uint8_to_nvs("storage", "device_id", 0);
            save_uint8_to_nvs("storage", "registration", 0);
            save_string_to_nvs("storage", "device_name", "");

            /* Save variable in storage */
            save_uint8_to_nvs("storage", "device_id", atoi(device_number));
            save_uint8_to_nvs("storage", "registration", 1);
            save_string_to_nvs("storage", "device_name", device_name);

            httpd_resp_send(req, "Configuration successfuly! Remove USB cable and restart device with battery.", HTTPD_RESP_USE_STRLEN);
        } else 
            httpd_resp_send(req, "Registration not successful! Retry please", HTTPD_RESP_USE_STRLEN);

        return ESP_OK;
    }

    httpd_resp_send_500(req);

    return ESP_FAIL;
}

/* Delete data page handler */
static esp_err_t delete_data_handler(httpd_req_t *req) {

    ESP_LOGI(TAG_WEBSERVER, "Delete data page handler");

    /* Clear configuration in storage */
    save_uint8_to_nvs("storage", "device_id", 0);
    save_uint8_to_nvs("storage", "registration", 2);
    save_string_to_nvs("storage", "device_name", "");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Configuration deleted successfully", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static httpd_uri_t home_page = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = home_page_handler,
    .user_ctx = NULL
};

static httpd_uri_t save_data = {
    .uri = "/save",
    .method = HTTP_POST,
    .handler = save_data_handler,
    .user_ctx = NULL
};

static httpd_uri_t delete_data = {
    .uri = "/delete",
    .method = HTTP_DELETE,
    .handler = delete_data_handler,
    .user_ctx = NULL
};

/* Configure access point esp32 door sensor */
esp_err_t wifi_init_softap() {

    esp_err_t err = ESP_FAIL;

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    err = esp_wifi_init(&cfg);
    if (err != ESP_OK){
        ESP_LOGE(TAG_WEBSERVER, "Error, wifi not inited");
        return err;
    }

    /* Set access point mode */
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK){
        ESP_LOGE(TAG_WEBSERVER, "Error, wifi mode not setted");
        return err;
    }

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_OPEN
        },
    };

    err = esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, wifi config not setted");
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, wifi not started");
        return err;
    }

    ESP_LOGI(TAG_WEBSERVER, "Access Point run. SSID:%s", WIFI_SSID);

    return err;
}

/* Start webserver */
esp_err_t start_webserver(httpd_handle_t server) {

    esp_err_t err = ESP_FAIL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, web server not started");
        return err;
    }

    err = httpd_register_uri_handler(server, &home_page);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, home_page function not registered");
        return err;
    }

    err = httpd_register_uri_handler(server, &save_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, save_data function not registered");
        return err;
    }

    err = httpd_register_uri_handler(server, &delete_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_WEBSERVER, "Error, delete_data function not registered");
        return err;
    }

    return err;
}

/* Stop webserver */
__attribute__((unused)) void stop_webserver(httpd_handle_t server) {

    if (!server) {
        ESP_LOGE(TAG_WEBSERVER, "Error, server not allocated");
        return;
    }        

    httpd_stop(server);

    return;
}