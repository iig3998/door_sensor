#ifndef ADC_H
#define ADC_H

#include <stdbool.h>

#pragma once

#define MAJOR_ADC_VER 0
#define MINOR_ADC_VER 1
#define PATCH_ADC_VER 0

void print_adc_version();

bool check_usb_connection();

bool check_status_battery();

#endif