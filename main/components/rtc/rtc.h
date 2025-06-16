#ifndef RTC_H
#define RTC_H

#include <stdint.h>

#pragma once

#define MAJOR_RTC_VER 0
#define MINOR_RTC_VER 1
#define PATCH_RTC_VER 0

void set_rtc_time(time_t time);

void print_current_time(time_t now);

#endif