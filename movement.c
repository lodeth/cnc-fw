#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

#include <avr/pgmspace.h>

#include "config.h"
#include "movement.h"
#include "protocol.h"
#include "serial.h"


extern const uint16_t pulse_timings[] PROGMEM;

extern struct position_block current_position;

// Used in pulse generation
static uint16_t intervalX;
static uint16_t intervalY;
static uint16_t intervalZ;

// ------------------------

// SIZE MUST BE A POWER OF TWO
#define OPERATION_QUEUE_SIZE 256
static uint8_t movement_queue_head;
static uint8_t movement_queue_tail;
static struct movement_block movement_queue_blocks[OPERATION_QUEUE_SIZE];

void movement_init()
{
    movement_stop();
    STEPPER_DDR = 0xff;
    STEPPER_PORT = 0xff;
    movement_disable_steppers();
    movement_disable_spindle();

    TCCR1A = 0;
    TCCR1C = 0;
    TCCR3A = 0;
    TCCR3C = 0;

    TCCR1B = (1<<CS10);
    TCCR3B = (1<<CS10);
}

void movement_stop()
{
    uint8_t flg = SREG;
    cli();
    TIMSK1 = 0;
    TIMSK3 = 0;
    movement_queue_head = 0;
    movement_queue_tail = 0;

    if (current_position.state & (1<<STATE_BIT_MOVING)) {
        current_position.state |= (1<<STATE_BIT_LOST);
    }

    current_position.state |= (1<<STATE_BIT_BUFFER_EMPTY);
    current_position.state &= ~(1<<STATE_BIT_BUFFER_FULL);
    current_position.state &= ~(1<<STATE_BIT_MOVING);
    current_position.state &= ~(1<<STATE_BIT_RUNNING);
    SREG = flg;
}

void movement_start()
{
    TCNT0 = 0;
    TCNT3 = 0;
    TIFR1 = 0;
    TIMSK1 = (1<<TOIE1);
}

int8_t movement_push(struct movement_block * op)
{
    uint8_t new_head = (movement_queue_head + 1) & (OPERATION_QUEUE_SIZE - 1);

    if (new_head == movement_queue_tail) {
        current_position.state |= (1<<STATE_BIT_BUFFER_FULL);
        return -1;
    }

    memcpy(&movement_queue_blocks[movement_queue_head], op, sizeof(struct movement_block));
    movement_queue_head = new_head;
    current_position.state &= ~(1<<STATE_BIT_BUFFER_EMPTY);
    return 0;
}

ISR(TIMER1_OVF_vect)
{
    if (movement_queue_head == movement_queue_tail) {
        current_position.state |= (1<<STATE_BIT_BUFFER_EMPTY);
        if (current_position.state & (1<<STATE_BIT_RUNNING)) {
            serial_tx(MSG_BUFFER_UNDERRUN);
            movement_stop();
        }
        return;
    }

    movement_jog(&movement_queue_blocks[movement_queue_tail]);
    current_position.state |= (1<<STATE_BIT_RUNNING);
    TIMSK1 |= (1<<TOIE1);

    movement_queue_tail++;
    if (movement_queue_tail == OPERATION_QUEUE_SIZE)
        movement_queue_tail = 0;
    current_position.state &= ~(1<<STATE_BIT_BUFFER_FULL);
}


void movement_jog(struct movement_block * next_op)
{
    TIMSK1 = 0;
    intervalX = pgm_read_word(&pulse_timings[abs(next_op->nX)]);
    OCR1A = intervalX;

    intervalY = pgm_read_word(&pulse_timings[abs(next_op->nY)]);
    OCR1B = intervalY;

    intervalZ = pgm_read_word(&pulse_timings[abs(next_op->nZ)]);
    OCR1C = intervalZ;

    uint8_t newport = (STEPPER_PORT & (0x80 | 0x40)) | 0x7;
    if (next_op->nX < 0)
        newport |= ((1<<3) << AXIS_BIT_X);
    if (next_op->nY < 0)
        newport |= ((1<<3) << AXIS_BIT_Y);
    if (next_op->nZ < 0)
        newport |= ((1<<3) << AXIS_BIT_Z);
    STEPPER_PORT = newport;

    if (next_op->nX != 0)
        TIMSK1 |= (1<<OCIE1A);
    if (next_op->nY != 0)
        TIMSK1 |= (1<<OCIE1B);
    if (next_op->nZ != 0)
        TIMSK1 |= (1<<OCIE1C);

    if (TIMSK1 != 0)
        current_position.state |= (1<<STATE_BIT_MOVING);
    else
        current_position.state &= ~(1<<STATE_BIT_MOVING);
}

// Take step on X
ISR(TIMER1_COMPA_vect)
{
    if (STEPPER_PORT & ((1<<3)<<AXIS_BIT_X)) {
        current_position.pX--;
    } else {
        current_position.pX++;
    }

    OCR1A += intervalX;
    STEPPER_PORT &= ~(1<<AXIS_BIT_X);
    OCR3A = TCNT3 + STEP_PULSE_WIDTH_TICKS;
    TIMSK3 |= (1<<OCIE3A);
}

// Take step on Y
ISR(TIMER1_COMPB_vect)
{
    if (STEPPER_PORT & ((1<<3)<<AXIS_BIT_Y)) {
        current_position.pY--;
    } else {
        current_position.pY++;
    }
    STEPPER_PORT &= ~(1<<AXIS_BIT_Y);
    OCR1B += intervalY;
    OCR3B = TCNT3 + STEP_PULSE_WIDTH_TICKS;
    TIMSK3 |= (1<<OCIE3B);
}

// Take step on Z
ISR(TIMER1_COMPC_vect)
{
    if (STEPPER_PORT & ((1<<3)<<AXIS_BIT_Z)) {
        current_position.pZ--;
    } else {
        current_position.pZ++;
    }

    STEPPER_PORT &= ~(1<<AXIS_BIT_Z);
    OCR1C += intervalZ;
    OCR3C = TCNT3 + STEP_PULSE_WIDTH_TICKS;
    TIMSK3 |= (1<<OCIE3C);
}

void movement_enable_steppers()
{
    STEPPER_PORT &= ~0x40;
}

void movement_disable_steppers()
{
    STEPPER_PORT |= 0x40;
    if (current_position.state & (1<<STATE_BIT_MOVING)) {
        current_position.state |= (1<<STATE_BIT_LOST);
    }
}

void movement_enable_spindle()
{
    STEPPER_PORT &= ~0x80;
}

void movement_disable_spindle()
{
    STEPPER_PORT |= 0x80;
}

