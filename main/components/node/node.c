#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "common.h"
#include "node.h"

#define TAG_NODE "NODE"

/* Print version library */
void print_node_version() {

    ESP_LOGI(TAG_NODE, "Node version: %u.%u.%u", MAJOR_NODE_VER, MINOR_NODE_VER, PATCH_NODE_VER);

    return;
}

/* Build node msg */
node_msg_t build_node_msg(cmd_type cmd, uint8_t id_node, node_type node, uint8_t id_msg, uint8_t mac[], const char *name_node, void *payload) {

    node_msg_t msg;

    memset(&msg, 0, sizeof(msg));

    msg.header.cmd = cmd;
    msg.header.id_node = id_node;
    msg.header.id_msg = id_msg;
    msg.header.node = node;

    /* Set mac address */
    memcpy(msg.header.mac, mac, MAC_SIZE);

    /* Set name node */
    memset(msg.name_node, '\0', NAME_LEN);
    memcpy(msg.name_node, name_node, strlen(name_node));

    /* Set payload */
    memset(msg.payload, 0, PAYLOAD_LEN);
    if (payload) {
        if (cmd == ADD) {
            memcpy(msg.payload, payload, sizeof(time_t));
        } else {
            memcpy(msg.payload, payload, sizeof(status_node));
            ESP_LOGI(TAG_NODE, "State: %u", msg.payload[0]);
            ESP_LOGI(TAG_NODE, "Battery low detect: %u", msg.payload[1]);
        }
    }

    msg.crc = calc_crc16_msg((uint8_t *)&msg, sizeof(msg) - sizeof(msg.crc));

    ESP_LOGI(TAG_NODE, "Cmd: %u", msg.header.cmd);
    ESP_LOGI(TAG_NODE, "ID node: %u", msg.header.id_node);
    ESP_LOGI(TAG_NODE, "ID msg: %u", msg.header.id_msg);
    ESP_LOGI(TAG_NODE, "Name: %s", msg.name_node);
    ESP_LOGI(TAG_NODE, "Mac: %02X:%02X:%02X:%02X:%02X:%02X", msg.header.mac[0], msg.header.mac[1], msg.header.mac[2], msg.header.mac[3], msg.header.mac[4], msg.header.mac[5]);
    ESP_LOGI(TAG_NODE, "CRC16: %u", msg.crc);

    return msg;
}