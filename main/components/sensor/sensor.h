#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#include "header.h"

#define MAC_SIZE 6

/* Struct node sensor */
typedef struct {
	node_id_header header;
	bool state;
	bool battery_low_detect;
	uint16_t crc;
} __attribute__((__packed__)) node_sensor_msg;

typedef struct {
	uint8_t id_node;
	uint8_t mac[MAC_SIZE];
	bool state;
	bool battery_low_detect;
} __attribute__((__packed__)) node_sensor;

/* Struct node alarm and siren list */
struct node_sensors_list {
	node_sensor node;
	struct node_sensors_list *next;
	struct node_sensors_list *prev;
} __attribute__((__packed__));

struct node_sensors_list *add_sensors_to_list(struct node_sensors_list *p, node_sensor_msg pn);

struct node_sensors_list *del_sensors_from_list(struct node_sensors_list *p, uint8_t id);

struct node_sensors_list *update_sensors_to_list(struct node_sensors_list *p, node_sensor_msg pn);

struct node_sensors_list *get_sensors_from_list(struct node_sensors_list *p, uint8_t id);

uint8_t get_num_sensors_from_list();

void print_sensors_list(struct node_sensors_list *p);

node_sensor_msg build_request_add_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[], bool state, bool battery_low_detect);

node_sensor_msg build_request_update_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[], bool state, bool battery_low_detect);

node_sensor_msg build_response_ack_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

node_sensor_msg build_response_nack_sensor_msg(uint8_t id_node, uint8_t id_msg, uint8_t mac[]);

#endif