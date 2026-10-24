#ifndef PRESENCE_NODE_H
#define PRESENCE_NODE_H

#include <stdint.h>

/** Node ID reported in RuView edge packets (last byte of the station MAC). */
uint8_t presence_node_id(void);

#endif /* PRESENCE_NODE_H */
