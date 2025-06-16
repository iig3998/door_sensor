#include <sys/time.h>

#include "esp_log.h"
#include "rtc.h"

// double  difftime (time_t time1, time_t time0);
#define TAG_RTC "RTC"

/* Set rtc time */
void set_rtc_time(time_t time) {

    struct timeval tv = {
        .tv_sec = time,
        .tv_usec = 0
    };

    settimeofday(&tv, NULL);
    
    ESP_LOGI(TAG_RTC, "Set time to %lld (UNIX timestamp)", time);

    return;
}

/* Print current time */
void print_current_time(time_t now) {
    
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_LOGI(TAG_RTC, "Current time: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    return;
}