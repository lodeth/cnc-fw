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
uint8_t movement_queue_head;
uint8_t movement_queue_tail;
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

    STEPPER_DDR = (1 << STEPPER_PIN_STEP_X) | (1 << STEPPER_PIN_STEP_Y) | (1 << STEPPER_PIN_STEP_Z) | (1 << STEPPER_PIN_STEP_A) |
                  (1 << STEPPER_PIN_DIR_X) | (1 << STEPPER_PIN_DIR_Y) | (1 << STEPPER_PIN_DIR_Z) | (1 << STEPPER_PIN_DIR_A);
    pins_disable_spindle();
    pins_disable_steppers();
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

int16_t movement_push(struct movement_block * op)
{
    uint8_t new_head = (movement_queue_head + 1) & (MOVEMENT_QUEUE_SIZE - 1);
    if (new_head == movement_queue_tail)
        return -1;
    memcpy(&movement_queue_blocks[movement_queue_head], op, sizeof(struct movement_block));
    status.free_slots--;
    uint8_t tag = movement_queue_head;
    movement_queue_head = new_head;
    STATE_FLAGS &= ~(1 << STATE_BIT_BUFFER_EMPTY);
     return tag;
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
    TIFR1 = 0;
    SYS_FLAGS_A |= (1 << EVENT_TIMESLICE);

    register uint8_t port = STEPPER_PORT;
    if ((port & (1 << STEPPER_PIN_DIR_X)) == 0) {
        status.X += steps_taken_X;
    } else {
        status.X -= steps_taken_X;
    }
    steps_taken_X = 0;

    if ((port & (1 << STEPPER_PIN_DIR_Y)) == 0) {
        status.Y += steps_taken_Y;
    } else {
        status.Y -= steps_taken_Y;
    }
    steps_taken_Y = 0;

    if ((port & (1 << STEPPER_PIN_DIR_Z)) == 0) {
        status.Z += steps_taken_Z;
    } else {
        status.Z -= steps_taken_Z;
    }
    steps_taken_Z = 0;
    if (STATE_FLAGS & (1 << STATE_BIT_ESTOP))
        return;


    if (STATE_FLAGS & (1 << STATE_BIT_RUNNING)) {
        if (movement_queue_head != movement_queue_tail) {
            movement_set(&movement_queue_blocks[movement_queue_tail]);
            status.tag = movement_queue_tail;
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
}

int8_t movement_jog(uint8_t axis, int8_t speed, uint8_t immediate)
{
    if (STATE_FLAGS & (1 << STATE_BIT_RUNNING))
        movement_stop(STOP_REASON_NONE);

    if ((STATE_FLAGS & (1 << STATE_BIT_MOVING)) == 0) {
        pins_enable_steppers();    
        memset(&jog_movement, 0, sizeof(struct movement_block));
    }

    if (speed > MAX_JOG_RATE)
        speed = MAX_JOG_RATE;
    else if (speed < -MAX_JOG_RATE)
        speed = -MAX_JOG_RATE;

    if (axis == 0) {
        if (immediate) jog_movement.X = speed;
        jog_target_speed.X = speed;
    } else if (axis == 1) {
        if (immediate) jog_movement.Y = speed;
        jog_target_speed.Y = speed;
    } else if (axis == 2) {
        if (immediate) jog_movement.Z = speed;
        jog_target_speed.Z = speed;
    }
    else
        return -1;

    STATE_FLAGS |= (1 << STATE_BIT_MOVING);
    return 0;
}

static void movement_set(struct movement_block * next_op)
{
    TIMSK1 = 0;
    TIFR1 = 0;
    status.stop_reason = STOP_REASON_NONE;

    status.vX = next_op->X;
    if (status.vX != 0) {
        if (status.vX > 0) {
            STEPPER_PORT &= ~(1 << STEPPER_PIN_DIR_X);
        } else {
            STEPPER_PORT |= (1 << STEPPER_PIN_DIR_X);
        }
        intervalX = pgm_read_word(&pulse_timings[abs(status.vX)]);
        OCR1A = intervalX;
        TIMSK1 |= (1 << OCIE1A);
    }

    status.vY = next_op->Y;
    if (status.vY != 0) {
        if (status.vY > 0) {
            STEPPER_PORT &= ~(1 << STEPPER_PIN_DIR_Y);
        } else {
            STEPPER_PORT |= (1 << STEPPER_PIN_DIR_Y);
        }
        intervalY = pgm_read_word(&pulse_timings[abs(status.vY)]);
        OCR1B = intervalY;
        TIMSK1 |= (1 << OCIE1B);
    }

    status.vZ = next_op->Z;
    if (status.vZ != 0) {
        if (status.vZ > 0) {
            STEPPER_PORT &= ~(1 << STEPPER_PIN_DIR_Z);
        } else {
            STEPPER_PORT |= (1 << STEPPER_PIN_DIR_Z);
        }
        intervalZ = pgm_read_word(&pulse_timings[abs(status.vZ)]);
        OCR1C = intervalZ;
        TIMSK1 |= (1 << OCIE1C);
    }

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
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_X);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_X);
#endif
    OCR3A = TCNT3 + STEP_PULSE_WIDTH_TICKS;
    OCR1A += intervalX;
    steps_taken_X++;
}

/*
ISR(TIMER3_COMPA_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_X);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_X);
#endif
}
*/

// Take step on Y
ISR(TIMER1_COMPB_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Y);
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_A);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Y);
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_A);
#endif

    OCR1B += intervalY;
    steps_taken_Y++;
    OCR3B = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

/*
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
*/

// Take step on Z
ISR(TIMER1_COMPC_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Z);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Z);
#endif

    OCR1C += intervalZ;
    steps_taken_Z++;
    OCR3C = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

/*
ISR(TIMER3_COMPC_vect)
{
#ifdef STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_STEP_Z);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_STEP_Z);
#endif
}
*/
