#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_err.h"

#define SSID_NAME "ESP32_Alarm_Sensor"

#define TAG_WEB_SERVER "WEB_SERVER"

static uint8_t device_id = 0;
static char device_name[32];

/* Funzione per gestire le richieste GET alla root */
static esp_err_t root_get_handler(httpd_req_t *req) {
    
    ESP_LOGI(TAG_WEB_SERVER, "%s", __func__);

    char response[650];
    memset(response, '\0', sizeof(response));

    snprintf(response, sizeof(response),
        "<!DOCTYPE html>"
        "<html>"
        "<body>"
        "<h2>Configura il tuo dispositivo</h2>"
        "<form action=\"/save\" method=\"post\">"
        "  ID dispositivo (0-255): <input type=\"number\" name=\"id\" value=\"%u\" min=\"0\" max=\"255\"><br><br>"
        "  Nome dispositivo: <input type=\"text\" name=\"name\" value=\"%s\"><br><br>"
        "  <input type=\"submit\" value=\"Salva\">"
        "</form>"
        "<script>"
        "document.querySelector('form').onsubmit = function(event) {"
        "  var id = parseInt(document.querySelector('[name=id]').value);"
        "  if (isNaN(id) || id < 0 || id > 255) {"
        "    alert('L\'ID deve essere un numero tra 0 e 255.');"
        "    event.preventDefault();"
        "  }"
        "};"
        "</script>"
        "</body>"
        "</html>",
        device_id, device_name);

    char on_resp[] = "<!DOCTYPE html><html><head><style type=\"text/css\">html {  font-family: Arial;  display: inline-block;  margin: 0px auto;  text-align: center;}h1{  color: #070812;  padding: 2vh;}.button {  display: inline-block;  background-color: #b30000; //red color  border: none;  border-radius: 4px;  color: white;  padding: 16px 40px;  text-decoration: none;  font-size: 30px;  margin: 2px;  cursor: pointer;}.button2 {  background-color: #364cf4; //blue color}.content {   padding: 50px;}.card-grid {  max-width: 800px;  margin: 0 auto;  display: grid;  grid-gap: 2rem;  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));}.card {  background-color: white;  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);}.card-title {  font-size: 1.2rem;  font-weight: bold;  color: #034078}</style>  <title>ESP32 WEB SERVER</title>  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">  <link rel=\"icon\" href=\"data:,\">  <link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.7.2/css/all.css\"    integrity=\"sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr\" crossorigin=\"anonymous\">  <link rel=\"stylesheet\" type=\"text/css\" ></head><body>  <h2>ESP32 WEB SERVER</h2>  <div class=\"content\">    <div class=\"card-grid\">      <div class=\"card\">        <p><i class=\"fas fa-lightbulb fa-2x\" style=\"color:#c81919;\"></i>     <strong>GPIO2</strong></p>        <p>GPIO state: <strong> ON</strong></p>        <p>          <a href=\"/led2on\"><button class=\"button\">ON</button></a>          <a href=\"/led2off\"><button class=\"button button2\">OFF</button></a>        </p>      </div>    </div>  </div></body></html>";

    httpd_resp_send(req, on_resp, strlen(on_resp));

    return ESP_OK;
}

static esp_err_t save_post_handler(httpd_req_t *req) {
    
    ESP_LOGI(TAG_WEB_SERVER, "%s", __func__);

    char content[200];
    int ret;
    int id_value = 0;
    
    memset(content, '\0', sizeof(content));
    ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    content[ret] = '\0';

    char *id = strstr(content, "id=");
    char *name = strstr(content, "name=");

    if (id) {
        id += strlen("id=");
        char *ampersand = strchr(id, '&');
        if (ampersand) *ampersand = '\0';
        id_value = atoi(id);
        if (id_value < 0 || id_value > 255) {
            ESP_LOGE(TAG_WEB_SERVER, "ID non valido: %d", id_value);
            httpd_resp_send(req, "Errore: L'ID deve essere un numero tra 0 e 255.", HTTPD_RESP_USE_STRLEN);
            return ESP_FAIL;
        }
        device_id = (uint8_t)id_value;
    }

    if(name) {
        name += strlen("name=");
        strncpy(device_name, name, sizeof(device_name) - 1);
        device_name[sizeof(device_name) - 1] = '\0';
    }

    ESP_LOGI(TAG_WEB_SERVER, "ID salvato: %u", device_id);
    ESP_LOGI(TAG_WEB_SERVER, "Nome salvato: %s", device_name);

    /* Response fortm web server */
    httpd_resp_send(req, "ID e Nome salvati con successo! Puoi chiudere questa pagina.", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* Configura e avvia il server HTTP */
httpd_handle_t start_webserver(void) {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    /* Run server */
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t save = {
            .uri       = "/save",
            .method    = HTTP_POST,
            .handler   = save_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &save);

        return server;
    }

    ESP_LOGE(TAG_WEB_SERVER, "Errore nell'avvio del server!");

    return NULL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG_WEB_SERVER, "station join, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG_WEB_SERVER, "station leave, AID=%d, reason=%d", event->aid, event->reason);
    }
}

/* Configure WiFi in access point */
void wifi_init_softap(void) {

    esp_netif_init();

    esp_event_loop_create_default();

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID_NAME,
            .ssid_len = strlen(SSID_NAME),
            .channel = 5,
            .password = "12345678",
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

    /* Start AP wifi */
    esp_wifi_start();

    return;
}

#if 0
/**/
void app_main(void) {

    esp_err_t ret = ESP_FAIL;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    if (ret != ESP_OK) {
        return;
    }

    /* Configura il Wi-Fi */
    wifi_init_softap();

    /* Run webserver */
    start_webserver();

    /* Aspetta che l'utente inserisca l'ID e il Nome */
    while (device_id == 0 && strlen(device_name) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG_WEB_SERVER, "Configurazione completata - ID: %u, Nome: %s", device_id, device_name);
}
#endif
