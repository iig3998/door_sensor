#include "esp_log.h"
#include "esp_err.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"

#include "led.h"

#define LED_STRIP_GPIO 7
#define TAG_LED        "LED"

static led_strip_handle_t led_strip = NULL;

/* Configure led driver */
esp_err_t init_led_driver() {

    esp_err_t err = ESP_FAIL;

    rmt_channel_handle_t rmt_channel;

    /* Confiugre rmt driver */
    rmt_tx_channel_config_t rmt_config = {
        .gpio_num = LED_STRIP_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000000,      // Set clock 10 MHz
        .mem_block_symbols = 48,
        .trans_queue_depth = 4,
    };

    err = rmt_new_tx_channel(&rmt_config, &rmt_channel);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, rmt tx channel not configured");
        return err;
    }

    /* Configure led driver */
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_led_config = {
        .resolution_hz = 10 * 1000000,
    };

    err = led_strip_new_rmt_device(&strip_config, &rmt_led_config, &led_strip);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, rmt led not configured");
        return err;
    }
    
    err = led_strip_clear(led_strip);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, grb led not cleared");
        return err;
    }

    return err;
}

/* Set led color */
void set_color_led(uint8_t color[]) {

    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG_LED, "Set color GRB");

    /* Set led pixel color */
    err = led_strip_set_pixel(led_strip, 0, color[0], color[1], color[2]);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, color not set");
        return;
    }

    /* Refresh led */
    err = led_strip_refresh(led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, led not refreshed");
        return;
    }

    return;
}

/**/
void power_off_led(void) {

    esp_err_t err = ESP_FAIL;
    
    err = led_strip_clear(led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, impossible to power off led");
    }

    return;
}

/**/
void deinit_led_driver() {
    esp_err_t err = ESP_FAIL;
    
    err = led_strip_del(led_strip);
    if(err != ESP_OK) {
        ESP_LOGE(TAG_LED, "Error, led not free");
    }

    return;
}
