#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void serial_init();

uint8_t serial_poll(); // Returns nonzero if bytes available
uint8_t serial_rx();
void serial_rxb(void * ptr, uint8_t len);

void serial_tx(uint8_t byte);
void serial_txb(void * ptr, uint8_t len);

#endif
