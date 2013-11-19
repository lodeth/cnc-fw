#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

#include <avr/pgmspace.h>

#include "config.h"
#include "movement.h"
#include "protocol.h"
#include "serial.h"
#include "config.h"
#include "pins.h"

#include "precalc.h"

extern struct status_block status;

// ------------------------
static uint8_t movement_queue_head;
static uint8_t movement_queue_tail;
static struct movement_block movement_queue_blocks[MOVEMENT_QUEUE_SIZE];


// Count for steps actually taken
static uint8_t steps_taken_X = 0;
static uint8_t steps_taken_Y = 0;
static uint8_t steps_taken_Z = 0;

// Used in pulse generation
static uint16_t intervalX;
static uint16_t intervalY;
static uint16_t intervalZ;

static void movement_set(struct movement_block * next_op);

void movement_init()
{
    TIMSK1 = (1 << TOIE1);
    TIFR1  = 0;

    pins_disable_spindle();
    pins_disable_steppers();

    STEPPER_DDR = (1 << STEPPER_PIN_STEP_X) | (1 << STEPPER_PIN_STEP_Y) | (1 << STEPPER_PIN_STEP_Z) | (1 << STEPPER_PIN_STEP_A) |
                  (1 << STEPPER_PIN_DIR_X) | (1 << STEPPER_PIN_DIR_Y) | (1 << STEPPER_PIN_DIR_Z) | (1 << STEPPER_PIN_DIR_A);
    movement_stop(STOP_REASON_RESET);

#ifndef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT = 0;
#else
    STEPPER_PORT = (1 << STEPPER_PIN_STEP_X) | (1 << STEPPER_PIN_STEP_Y) | (1 << STEPPER_PIN_STEP_Z) | (1 << STEPPER_PIN_STEP_A);
#endif

    TCCR1A = 0;
    TCCR3A = 0;
    TCCR1B = 0;
    TCCR3B = 0;
    TCCR1B = MOVEMENT_TIMESLICE;
    TCCR3B = MOVEMENT_TIMESLICE; 
}

void movement_stop(uint8_t reason)
{
    uint8_t flg = SREG;
    cli();

    TIMSK1 = (1 << TOIE1);
    TIFR1  = 0;

    movement_queue_head = 0;
    movement_queue_tail = 0;
    status.stop_reason = reason;
    status.free_slots  = MOVEMENT_QUEUE_SIZE - 1;

    if (reason == STOP_REASON_ESTOP) {
        STATE_FLAGS |= (1 << STATE_BIT_ESTOP);
        pins_disable_spindle();
#ifdef STEPPERS_DISABLED_IN_ESTOP
        pins_disable_steppers();
#endif
    }

    STATE_FLAGS |=  (1 << STATE_BIT_BUFFER_EMPTY);
    STATE_FLAGS &= ~(1 << STATE_BIT_RUNNING);
    STATE_FLAGS &= ~(1 << STATE_BIT_MOVING);
    SREG = flg;
}

void movement_cycle_start()
{
    pins_enable_steppers();    
    TCNT0 = 0;
    TCNT3 = 0;
    TIFR1 = 0;

    STATE_FLAGS |= (1 << STATE_BIT_RUNNING);
    status.stop_reason = STOP_REASON_NONE;
}

struct movement_block * movement_queue_get_free()
{
    uint8_t new_head = (movement_queue_head + 1) & (MOVEMENT_QUEUE_SIZE - 1);
    if (new_head == movement_queue_tail)
        return NULL;
    return &movement_queue_blocks[movement_queue_head];
}

uint8_t movement_queue_commit()
{
    status.free_slots--;
    uint8_t tag = movement_queue_head;
    movement_queue_head = (movement_queue_head + 1) & (MOVEMENT_QUEUE_SIZE - 1);
    STATE_FLAGS &= ~(1 << STATE_BIT_BUFFER_EMPTY);
    return tag;
}

static struct movement_block jog_target_speed;
static struct movement_block jog_movement;

ISR(TIMER1_OVF_vect)
{
    TIMSK1 = (1 << TOIE1);

    if (STATE_FLAGS & (1 << STATE_BIT_RUNNING)) {
        if (movement_queue_head != movement_queue_tail) {
            status.tag = movement_queue_tail;
            movement_set(&movement_queue_blocks[movement_queue_tail]);
            movement_queue_tail = (movement_queue_tail + 1) & (MOVEMENT_QUEUE_SIZE - 1);
            status.free_slots++;
        } else {
            STATE_FLAGS |= (1 << STATE_BIT_BUFFER_EMPTY);
            if (STATE_FLAGS & (1 << STATE_BIT_RUNNING)) {
                movement_stop(STOP_REASON_UNDERRUN);
            }
        }
    } else if (STATE_FLAGS & (1 << STATE_BIT_MOVING)) {
        int16_t d;
        d = (int16_t) jog_target_speed.X - jog_movement.X;
        if (d < 0)
            jog_movement.X--;
        else if (d > 0)
            jog_movement.X++;
        d = (int16_t) jog_target_speed.Y - jog_movement.Y;
        if (d < 0)
            jog_movement.Y--;
        else if (d > 0)
            jog_movement.Y++;
        d = (int16_t) jog_target_speed.Z - jog_movement.Z;
        if (d < 0)
            jog_movement.Z--;
        else if (d > 0)
            jog_movement.Z++;
        movement_set(&jog_movement);
    }

    SYS_FLAGS_A |= (1 << EVENT_TIMESLICE);
}

int8_t movement_jog(struct movement_block * op)
{
    if (STATE_FLAGS & (1 << STATE_BIT_RUNNING))
        movement_stop(STOP_REASON_NONE);

    if ((STATE_FLAGS & (1 << STATE_BIT_MOVING)) == 0) {
        pins_enable_steppers();    
        memset(&jog_movement, 0, sizeof(struct movement_block));
    }
    asm volatile("cli");
    memcpy(&jog_target_speed, op, sizeof(struct movement_block));
    asm volatile("sei");
    STATE_FLAGS |= (1 << STATE_BIT_MOVING);
    return 0;
}

static void movement_set(struct movement_block * next_op)
{
    register uint8_t port = STEPPER_PORT;
#ifdef STEPPER_DIR_POSITIVE_LOW
    port ^= (1 << STEPPER_PIN_DIR_X) | (1 << STEPPER_PIN_DIR_Y) | (1 << STEPPER_PIN_DIR_Z) | (1 << STEPPER_PIN_DIR_A);
#endif

    if (port & (1 << STEPPER_PIN_DIR_X)) {
        status.X += steps_taken_X;
    } else {
        status.X -= steps_taken_X;
    }

    if (port & (1 << STEPPER_PIN_DIR_Y)) {
        status.Y += steps_taken_Y;
    } else {
        status.Y -= steps_taken_Y;
    }

    if (port & (1 << STEPPER_PIN_DIR_Z)) {
        status.Z += steps_taken_Z;
    } else {
        status.Z -= steps_taken_Z;
    }

    steps_taken_X = 0;
    steps_taken_Y = 0;
    steps_taken_Z = 0;

    if (STATE_FLAGS & (1 << STATE_BIT_ESTOP))
        return;

    TIMSK1 = 0;
    status.stop_reason = STOP_REASON_NONE;
    intervalX = pulse_timings[abs(next_op->X)];
    intervalY = pulse_timings[abs(next_op->Y)];
    intervalZ = pulse_timings[abs(next_op->Z)];

    OCR1A = intervalX;
    OCR1B = intervalY;
    OCR1C = intervalZ;
    TIFR1 = 0;

    register uint8_t newport = 0;
#ifdef STEPPER_STEP_ACTIVE_LOW
    newport |= (1 << STEPPER_PIN_STEP_X) | (1 << STEPPER_PIN_STEP_Y) | (1 << STEPPER_PIN_STEP_Z) | (1 << STEPPER_PIN_STEP_A);
#endif

    if (next_op->X != 0) {
        if (next_op->X > 0)
            newport |= (1 << STEPPER_PIN_DIR_X);
        TIMSK1 |= (1 << OCIE1A);
    }
    if (next_op->Y != 0) {
        if (next_op->Y > 0)
            newport |= (1 << STEPPER_PIN_DIR_Y) | (1 << STEPPER_PIN_DIR_A);
        TIMSK1 |= (1 << OCIE1B);
    }
    if (next_op->Z != 0) {
        if (next_op->Z > 0)
            newport |= (1 << STEPPER_PIN_DIR_Z);
        TIMSK1 |= (1 << OCIE1C);
    }

#ifdef STEPPER_DIR_POSITIVE_LOW
    newport ^= (1 << STEPPER_PIN_DIR_X) | (1 << STEPPER_PIN_DIR_Y) | (1 << STEPPER_PIN_DIR_Z) | (1 << STEPPER_PIN_DIR_A);
#endif

    STEPPER_PORT = newport;

    if (TIMSK1 != 0)
        STATE_FLAGS |= (1 << STATE_BIT_MOVING);
    else
        STATE_FLAGS &= ~(1 << STATE_BIT_MOVING);

    TIMSK3 = TIMSK1;
    TIFR1 = TIMSK1;
    TIMSK1 |= (1 << TOIE1);
}

// Take step on X
ISR(TIMER1_COMPA_vect)
{
    OCR1A += intervalX;
    steps_taken_X++;
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_X);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_X);
#endif
    OCR3A = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

ISR(TIMER3_COMPA_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_X);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_X);
#endif
}

// Take step on Y
ISR(TIMER1_COMPB_vect)
{
    OCR1B += intervalY;
    steps_taken_Y++;
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Y);
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_A);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Y);
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_A);
#endif
    OCR3B = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

ISR(TIMER3_COMPB_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Y);
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_A);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Y);
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_A);
#endif
}

// Take step on Z
ISR(TIMER1_COMPC_vect)
{
    OCR1C += intervalZ;
    steps_taken_Z++;
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Z);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Z);
#endif
    OCR3C = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

ISR(TIMER3_COMPC_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Z);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Z);
#endif
}
