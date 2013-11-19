#ifndef PINS_H
#define PINS_H
#include <stdint.h>

void pins_init();
uint8_t pins_is_estop_line_asserted();
void pins_set_active_low_mask(uint8_t mask);

void pins_enable_steppers();
void pins_disable_steppers();
void pins_enable_spindle();
void pins_disable_spindle();

#endif

