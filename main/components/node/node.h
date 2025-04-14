#ifndef NODE_H
#define NODE_H

#include <stdint.h>
#include <stdbool.h>

#include "header.h"

#define MAC_SIZE 6
#define NAME_LEN 15

#define MAJOR_NODE_VER 0
#define MINOR_NODE_VER 1
#define PATCH_NODE_VER 0

/* Struct node */
typedef struct {
	node_id_header header;
	char name_node[NAME_LEN];
	void *payload;
	uint16_t crc;
} __attribute__((__packed__)) node_msg_t;

typedef struct {
	uint8_t id_node;
	uint8_t mac[MAC_SIZE];
	char name_node[NAME_LEN];
} __attribute__((__packed__)) node_t;

/* Struct node list */
struct node_list_t {
	node_t node;
	struct node_list_t *next;
	struct node_list_t *prev;
} __attribute__((__packed__));

void print_node_version();

struct node_list_t *add_node_to_list(struct node_list_t *p, node_msg_t pn);

struct node_list_t *del_node_from_list(struct node_list_t *p, uint8_t id);

struct node_list_t *update_node_to_list(struct node_list_t *p, node_msg_t pn);

struct node_list_t *get_node_from_list(struct node_list_t *p, uint8_t id);

uint8_t get_num_node_from_list();

node_msg_t build_cmd_node_msg(enum cmd_type cmd, uint8_t id_node, uint8_t id_msg, uint8_t mac[], const char *name_node, void *payload);

#endif