#ifndef MOVEMENT_H
#define MOVEMENT_H
#include <stdint.h>

struct movement_block; // Defined in protocol.h

void movement_init();

void movement_enable_steppers();
void movement_disable_steppers();
void movement_enable_spindle();
void movement_disable_spindle();

// stop movement and flush queues
void movement_stop(uint8_t reason);

// start executing queued movements
void movement_start();

// push movement to queue, returns -1 on queue full, otherwise returns tag
int16_t movement_push(struct movement_block * op);

// set movement speeds immediately, returns -1 if queue started
int8_t movement_jog(struct movement_block * op);
#endif

