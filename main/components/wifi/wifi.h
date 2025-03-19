#ifndef WIFI_H
#define WIFI_H

#pragma once

#define WIFI_MAJOR 0
#define WIFI_MINOR 1
#define WIFI_PATCH 0

#define TAG_WIFI "WIFI"
#define ESPNOW_CHANNEL 7

/* Print wifi version */
void print_wifi_version();

/* Init WiFi station */
esp_err_t init_wifi_sta();

void deinit_wifi_sta();

#endif