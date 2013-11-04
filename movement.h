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
void movement_stop();

// start executing queued movements
void movement_start();

// push movement to queue, returns -1 on queue full
int8_t movement_push(struct movement_block * op);

// set movement speeds immediately
void movement_jog(struct movement_block * op);
#endif

