#include <avr/io.h>
#include <avr/interrupt.h>

#include "pins.h"
#include "config.h"
#include "protocol.h"
#include "movement.h"

__attribute__((always_inline))
inline uint8_t pins_get_inputs()
{
    return INPUT_PIN ^ ( 
#ifdef INPUT_ESTOP_ACTIVE_LOW
    (1 << INPUT_ESTOP) | 
#endif
#ifdef INPUT_PROBE_ACTIVE_LOW
    (1 << INPUT_PROBE) | 
#endif
#ifdef INPUT_LIMITS_ACTIVE_LOW
    (1 << INPUT_ESTOP) | 
#endif
    0);
}

uint8_t pins_is_estop_line_asserted()
{
#ifdef INPUT_ESTOP_ACTIVE_LOW
    return (INPUT_PIN & (1 << INPUT_ESTOP)) == 0;
#else
    return (INPUT_PIN & (1 << INPUT_ESTOP)) != 0;
#endif
}

extern struct status_block status;

void pins_init()
{
    INPUT_DDR  = 0x00;
    INPUT_PORT = 0xff;
    INPUT_PCIMSK = 0xff;
    status.inputs = pins_get_inputs();
    PCICR |= (1 << INPUT_PCIE);
    PCIFR |= (1 << INPUT_PCIE);

    CONTROL_DDR |= (1 << CONTROL_PIN_ENABLE) | (1 << CONTROL_PIN_SPINDLE);
    pins_disable_spindle();
    pins_disable_steppers();
}

ISR(INPUT_PCI_vect)
{
    register uint8_t inputs = pins_get_inputs();
    uint8_t change = inputs ^ status.inputs;
    status.inputs = inputs;

    // Don't handle if we're in ESTOP already
    if (STATE_FLAGS & (1 << STATE_BIT_ESTOP))
        return;
    
    if ((inputs & (1 << INPUT_ESTOP))) {
        movement_stop(STOP_REASON_ESTOP);
        return; // We don't care about limits if ESTOP was just triggered
    }

    if (change & ((1 << INPUT_LIMIT_X) | (1 << INPUT_LIMIT_Y) | (1 << INPUT_LIMIT_Z))) {
        if (STATE_FLAGS & (1 << STATE_BIT_RUNNING)) {
            movement_stop(STOP_REASON_LIMIT);
        } else { // We're jogging, do partial automatic homing
            if (change & (1 << INPUT_LIMIT_X)) {
                movement_jog(0, 0, 1);
                status.X = 0;
            }
            if (change & (1 << INPUT_LIMIT_Y)) {
                movement_jog(1, 0, 1);
                status.Y = 0;
            }
            if (change & (1 << INPUT_LIMIT_Z)) {
                movement_jog(2, 0, 1);
                status.Z = 0;
            }
        }
    } else if (change & (1 << INPUT_PROBE))
        movement_stop(STOP_REASON_PROBE);
}

void pins_enable_steppers()
{
#ifdef CONTROL_ENABLE_ACTIVE_LOW
    CONTROL_PORT &= ~(1 << CONTROL_PIN_ENABLE);
#else
    CONTROL_PORT |= (1 << CONTROL_PIN_ENABLE);
#endif
}

void pins_disable_steppers()
{
#ifdef CONTROL_ENABLE_ACTIVE_LOW
    CONTROL_PORT |= (1 << CONTROL_PIN_ENABLE);
#else
    CONTROL_PORT &= ~(1 << CONTROL_PIN_ENABLE);
#endif
}

void pins_enable_spindle()
{
#ifdef CONTROL_SPINDLE_ACTIVE_LOW
    CONTROL_PORT &= ~(1 << CONTROL_PIN_SPINDLE);
#else
    CONTROL_PORT |= (1 << CONTROL_PIN_SPINDLE);
#endif
}

void pins_disable_spindle()
{
#ifdef CONTROL_SPINDLE_ACTIVE_LOW
    CONTROL_PORT |= (1 << CONTROL_PIN_SPINDLE);
#else
    CONTROL_PORT &= ~(1 << CONTROL_PIN_SPINDLE);
#endif
}

