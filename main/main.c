#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_WAKEUP_PIN  GPIO_NUM_25
#define DEBOUNCE_COUNTER 50
#define TAG_GPIO         "GPIO"

RTC_DATA_ATTR uint8_t new_state = 0;
RTC_DATA_ATTR uint8_t old_state = 0;

/* sotto i 2.6 volt della batteria allarme */
void app_main() {

    static uint8_t counter = DEBOUNCE_COUNTER;
    ESP_ERROR_CHECK(rtc_gpio_init(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_dis(GPIO_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(GPIO_WAKEUP_PIN, RTC_GPIO_MODE_INPUT_ONLY));

    /* Debounce filter */
    while(counter > 0) {
        new_state = rtc_gpio_get_level(GPIO_WAKEUP_PIN);
        if (new_state != old_state)
            counter = DEBOUNCE_COUNTER;
        else
            counter --;

        old_state = new_state;
    }

    if ((counter == 0) && (new_state == 0)) {
        ESP_LOGI(TAG_GPIO, "Open");
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 1));
    } else if ((counter == 0) && (new_state == 1)) {
        ESP_LOGI(TAG_GPIO, "Close");
        ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_WAKEUP_PIN, 0));
    }

    ESP_LOGI("DeepSleep", "Going to deep sleep. Wakeup by GPIO pin %d", GPIO_WAKEUP_PIN);

    vTaskDelay(5/portTICK_PERIOD_MS);

    /* Entra in deep sleep */
    esp_deep_sleep_start();

}