#include <malloc.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "common.h"
#include "node.h"

#define TAG_NODE "NODE"

static uint8_t counter_node = 0;

/* Print version library */
void print_node_version() {

    ESP_LOGI(TAG_NODE, "Node version: %u.%u.%u", MAJOR_NODE_VER, MINOR_NODE_VER, PATCH_NODE_VER);

    return;
}

/* Return tail node list */
static struct node_list_t *get_tail_node_list(struct node_list_t *p) {

    if(!p)
        return p;

    while(p->next) {
        p = p->next;
    }

    return p;
}

/* Return head node list */
static struct node_list_t *get_head_node_list(struct node_list_t *p) {

    if(!p)
        return p;

    while(p->prev) {
        p = p->prev;
    }

    return p;
}

/* Return node from list */
struct node_list_t *get_node_from_list(struct node_list_t *p, uint8_t id) {

    p = get_head_node_list(p);
    if (!p)
        return NULL;

    do {
        if(p->node.id_node == id) {
            return p;
        }

        p = p->next;

    } while(p->next != NULL);

    if(p->node.id_node == id) {
        return p;
    }

    return NULL;
}

/* Add node to list */
struct node_list_t *add_node_to_list(struct node_list_t *p, node_msg_t pn) {

    p = get_tail_node_list(p);

    struct node_list_t *px = (struct node_list_t *)calloc(1, sizeof(struct node_list_t));
    if(!px)
        return NULL;

    px->node.id_node = pn.header.id_node;
    memcpy(px->node.mac, pn.header.mac, MAC_SIZE);

    px->next = NULL;

    if(!p){
        px->prev = NULL;
    } else {
        px->prev = p;
        p->next = px;
    }

    p = px;
    counter_node++;

    return p;
}

/* Delete node from list */
struct node_list_t *del_node_from_list(struct node_list_t *p, uint8_t id) {

    struct node_list_t *p1 = NULL;
    struct node_list_t *p2 = NULL;
    struct node_list_t *p0 = NULL;

    p0 = get_node_from_list(p, id);
    if(!p0) {
        return p;
    }

    if(p0->prev)
        p1 = p0->prev;

    if(p0->next)
        p2 = p0->next;

    if(p1 && !p2)
        p1->next = NULL;

    if(!p1 && p2)
        p2->prev = NULL;

    if (p1 && p2) {
        p1->next = p2;
        p2->prev = p1;
    }

    free(p0);

    counter_node--;

    if(p2)
        return get_tail_node_list(p2);
    else if(p1)
        return get_tail_node_list(p1);

    return NULL;
}

/* Update node */
struct node_list_t *update_node_to_list(struct node_list_t *p, node_msg_t pn) {

    p = get_node_from_list(p, pn.header.id_node);
    if (!p)
        return p;

    memcpy(p, &pn, sizeof(pn));

    return p;
}

/* Return number of node inside list */
uint8_t get_num_node_from_list() {

    return counter_node;
}

node_msg_t build_cmd_node_msg(enum cmd_type cmd, uint8_t id_node, uint8_t id_msg, uint8_t mac[], const char* name_node, void *payload) {

    node_msg_t msg;

    msg.header.cmd = cmd;
    msg.header.id_node = id_node;
    msg.header.id_msg = id_msg;
    memcpy(msg.header.mac, mac, MAC_SIZE);

    msg.header.node = SENSOR;

    memcpy(msg.name_node, name_node, 15);

    if(!payload) {
        msg.payload = payload;
    }

    msg.crc = calc_crc16((uint8_t *)&msg, sizeof(msg) - sizeof(msg.crc));

    ESP_LOGI(TAG_NODE, "Cmd: %u", cmd);
    ESP_LOGI(TAG_NODE, "ID node: %u", id_node);
    ESP_LOGI(TAG_NODE, "ID msg: %u", id_msg);
    ESP_LOGI(TAG_NODE, "Name: %s", msg.name_node);
    ESP_LOGI(TAG_NODE, "CRC16: %u", msg.crc);

    return msg;
}