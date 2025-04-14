#ifndef HEADER_H
#define HEADER_H

#include <stdint.h>

#define MAJOR_HEADER_VER 0
#define MINOR_HEADER_VER 1
#define PATCH_HEADER_VER 0

#define MAC_SIZE 6

enum node_type {
    SENSOR = 0,
    SIREN,
	KEYBOARD,
	GATEWAY
};

enum cmd_type {
	ADD = 0,
	DEL,
	UPDATE,
	GET,
	ACTIVE,
	DEACTIVE,
	ACK,
	NACK
};

/* Header struct
+--------------------+-----------------+------------------+--------------+--------------+
| node_type (1 byte) | id_msg (1 byte) | id_node (1 byte) | mac (6 byte) | cmd (1 byte) |
+--------------------+-----------------+------------------+--------------+--------------+
*/

typedef struct {
	enum node_type node;
	uint8_t id_node;
	uint8_t id_msg;
	uint8_t mac[MAC_SIZE];
	enum cmd_type cmd;
} __attribute__((__packed__)) node_id_header;

node_id_header build_header_msg(enum node_type node, uint8_t id_node, uint8_t id_msg, uint8_t mac[], enum cmd_type cmd);

#endif