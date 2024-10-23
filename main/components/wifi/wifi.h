#ifndef WIFI_H
#define WIFI_H

#pragma once

#include <stdint.h>

#define WIFI_MAJOR 0
#define WIFI_MINOR 1
#define WIFI_PATCH 0

#ifdef __cplusplus
extern "C" {
#endif

/* Print wifi version */
void print_wifi_version();

/* Init WiFi */
void init_wifi();

/* Wait for the wifi connection to be established */
void wait_wifi_connection_established();

/* Get wifi status connectoin */
uint8_t get_wifi_status_connection();

#ifdef __cplusplus
}
#endif

#endif