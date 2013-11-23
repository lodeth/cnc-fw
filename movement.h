#ifndef MOVEMENT_H
#define MOVEMENT_H
#include <stdint.h>

struct movement_block; // Defined in protocol.h

void movement_init();

// stop movement and flush queues
void movement_stop(uint8_t reason);

// start executing queued movements
void movement_cycle_start();

// get free movement block, return NULL on full
struct movement_block * movement_queue_get_free();
// and mark it as filled and return tag
uint8_t movement_queue_commit();

int16_t movement_push(struct movement_block * op);

// set movement speeds, returns -1 if queue started
int8_t movement_jog(uint8_t axis, int8_t speed, uint8_t immediate);

#endif

