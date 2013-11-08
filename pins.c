#include <avr/io.h>
#include <avr/interrupt.h>

#include "pins.h"
#include "config.h"
#include "protocol.h"
#include "movement.h"

__attribute__((always_inline))
inline uint8_t pins_get_inverted_inputs()
{
    register uint8_t state = INPUT_PIN;
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_ESTOP))      { state ^= (1 << INPUT_ESTOP); }
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_LIMITS))     { state ^= (1 << INPUT_LIMIT_X); state ^= (1 << INPUT_LIMIT_Y); state ^= (1 << INPUT_LIMIT_Z); }
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_PROBE))      { state ^= (1 << INPUT_PROBE); }
    return state;
};


uint8_t pins_is_estop_line_asserted()
{
    register uint8_t ip = INPUT_PIN & (1 << INPUT_ESTOP);
    return (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_ESTOP)) ? (ip == 0) : (ip != 0);
}

void pins_set_active_low_mask(uint8_t mask)
{
    STATE_FLAGS |= (1 << STATE_BIT_ESTOP);
    SYS_FLAGS_A = mask | (SYS_FLAGS_A & (1 << EVENT_TIMESLICE));
    PCIFR |= (1 << INPUT_PCIE);
}

extern struct status_block status;

void pins_init_inputs()
{
    INPUT_DDR  = 0x00;
    INPUT_PORT = 0xff;
    INPUT_PCIMSK = 0xff;
    status.inputs = pins_get_inverted_inputs();
    PCICR |= (1 << INPUT_PCIE);
    PCIFR |= (1 << INPUT_PCIE);
}

ISR(INPUT_PCI_vect)
{
    register uint8_t inputs = pins_get_inverted_inputs();
    uint8_t change = inputs ^ status.inputs;
    status.inputs = inputs;

    // Don't handle if we're in ESTOP already
    if (STATE_FLAGS & (1 << STATE_BIT_ESTOP))
        return;
    
    if ((inputs & (1 << INPUT_ESTOP)) != 0) {
        movement_stop(STOP_REASON_ESTOP);
        return; // We don't care about limits if ESTOP was just triggered
    }

    if (change & ((1 << INPUT_LIMIT_X) | (1 << INPUT_LIMIT_Y) | (1 << INPUT_LIMIT_Z))) {
        movement_stop(STOP_REASON_LIMIT);
    } else if (change & (1 << INPUT_PROBE))
        movement_stop(STOP_REASON_PROBE);
}


void movement_enable_steppers()
{
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_ENABLE)) {
        STEPPER_PORT &= ~(1 << STEPPER_PIN_ENABLE);
    } else {
        STEPPER_PORT |= (1 << STEPPER_PIN_ENABLE);
    }
}

void movement_disable_steppers()
{
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_ENABLE)) {
        STEPPER_PORT |= (1 << STEPPER_PIN_ENABLE);
    } else {
        STEPPER_PORT &= ~(1 << STEPPER_PIN_ENABLE);
    }
}

void movement_enable_spindle()
{
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_SPINDLE)) {
        STEPPER_PORT &= ~(1 << STEPPER_PIN_SPINDLE);
    } else {
        STEPPER_PORT |= (1 << STEPPER_PIN_SPINDLE);
    }
}

void movement_disable_spindle()
{
    if (SYS_FLAGS_A & (1 << ACTIVE_LOW_BIT_SPINDLE)) {
        STEPPER_PORT |= (1 << STEPPER_PIN_SPINDLE);
    } else {
        STEPPER_PORT &= ~(1 << STEPPER_PIN_SPINDLE);
    }
}

