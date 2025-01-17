#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"

#include "sensor.h"

#define TAG_SENSOR "SENSOR"

/* Print node caratteristics */
static void print_node(node_id_alarm pkt) {

    ESP_LOGI(TAG_SENSOR, "Cmd: %u", pkt.header.cmd);
    ESP_LOGI(TAG_SENSOR, "ID: %u", pkt.header.id);
    ESP_LOGI(TAG_SENSOR, "Node type: %u", pkt.header.node);
    ESP_LOGI(TAG_SENSOR, "Sensor state: %u", pkt.state);
    ESP_LOGI(TAG_SENSOR, "Battery low detect: %u", pkt.battery_low_detect);
    ESP_LOGI(TAG_SENSOR, "Time: %lld", pkt.time);
    ESP_LOGI(TAG_SENSOR, "CRC16: %u", pkt.crc);

    return;
}

/* Calculate crc16 */
static uint16_t calc_crc16(uint8_t *data, uint32_t length) {

    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++) {

        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0; bit < 8; bit++)  {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8005;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* Alarm sensor update packet */
node_id_alarm alarm_sensor_update_pkt(uint8_t id, bool state, bool battery_low_detect, int64_t time) {

    node_id_alarm pkt;

    memset(&pkt, 0, sizeof(pkt));

    /* Build header packet */
    pkt.header.cmd = UPDATE;
    pkt.header.id = id;
    esp_read_mac(pkt.header.mac, ESP_MAC_WIFI_STA);
	pkt.header.node = ALARM_SENSOR;

    pkt.state = state;
    pkt.battery_low_detect = battery_low_detect;
	pkt.time = time;
	pkt.crc = calc_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

    print_node(pkt);

    return pkt;
}

/* Alarm sensor add packet */
node_id_alarm alarm_sensor_add_pkt(uint8_t id, int64_t time) {

    node_id_alarm pkt;

    memset(&pkt, 0, sizeof(pkt));

    /* Build header packet */
    pkt.header.cmd = ADD;
    pkt.header.id = id;
    esp_read_mac(pkt.header.mac, ESP_MAC_WIFI_STA);
	pkt.header.node = ALARM_SENSOR;

    pkt.state = false;
    pkt.battery_low_detect = false;
	pkt.time = time;
	pkt.crc = calc_crc16((uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc));

    print_node(pkt);

    return pkt;
}