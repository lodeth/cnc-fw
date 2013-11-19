#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include "config.h"

void serial_init();

// Returns nonzero if bytes available
uint8_t serial_poll();

uint8_t serial_rx();
void serial_tx(uint8_t byte);

void serial_rxb(void * ptr, uint8_t len);
void serial_txb(void * ptr, uint8_t len);

#endif
