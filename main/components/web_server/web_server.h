#ifndef WEB_SERVER
#define WEB_SERVER

#include "esp_http_server.h"



uint8_t get_id_device();

char *get_name_device();

bool is_device_configured();

esp_err_t wifi_init_sta_ws();

httpd_handle_t start_webserver();

void stop_webserver(httpd_handle_t server);

#endif