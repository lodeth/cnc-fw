#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

void serial_init();
void serial_tx(uint8_t byte);
void serial_txb(void * ptr, uint8_t len);

#endif
