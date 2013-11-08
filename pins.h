#ifndef PINS_H
#define PINS_H
#include <stdint.h>

void pins_init_inputs();
uint8_t pins_is_estop_line_asserted();
void pins_set_active_low_mask(uint8_t mask);

#endif

