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

#if STEPPER_DIR_POSITIVE_LOW
#define DIR_IS_POSITIVE(d) ((STEPPER_PORT & (1 << d)) == 0)
#define SHOULD_SET_DIR_BIT(d) (d) < 0
#else
#define DIR_IS_POSITIVE(d) (STEPPER_PORT & (1 << d))
#define SHOULD_SET_DIR_BIT(d) (d) > 0
#endif

#if STEPPER_STEP_ACTIVE_LOW
#define ACTIVATE_STEP(d) do {STEPPER_PORT &= ~(1 << d); } while(0)
#else
#define ACTIVATE_STEP(d) do {STEPPER_PORT |= (1 << d);} while(0)
#endif


extern const uint16_t pulse_timings[] PROGMEM;

extern struct position_block current_position;

// Used in pulse generation
static uint16_t intervalX;
static uint16_t intervalY;
static uint16_t intervalZ;

// ------------------------
static uint8_t movement_queue_head;
static uint8_t movement_queue_tail;
static struct movement_block movement_queue_blocks[MOVEMENT_QUEUE_SIZE];

void movement_set(struct movement_block * next_op);

void movement_init()
{
    movement_stop(STOP_REASON_RESET);
    STEPPER_DDR = 0xff;

    movement_disable_steppers();
    movement_disable_spindle();

#if STEPPER_STEP_ACTIVE_LOW
    STEPPER_PORT |= ((1 << STEPPER_PIN_STEP_X) |
                     (1 << STEPPER_PIN_STEP_Y) |
                     (1 << STEPPER_PIN_STEP_Z));
#else
    STEPPER_PORT &= ~((1 << STEPPER_PIN_STEP_X) |
                      (1 << STEPPER_PIN_STEP_Y) |
                      (1 << STEPPER_PIN_STEP_Z));
#endif

    TCCR1A = 0;
    TCCR1C = 0;
    TCCR3A = 0;
    TCCR3C = 0;
    TCNT0 = 0;
    TCNT3 = 0;

    TCCR1B = (MOVEMENT_TIMESLICE << CS10);
    TCCR3B = (1 << CS10);
}

void movement_stop(uint8_t reason)
{
    uint8_t flg = SREG;
    cli();
    TIMSK1 = 0;
    TIMSK3 = 0;
    movement_queue_head = 0;
    movement_queue_tail = 0;

    // It should be safe to assume that we're not moving fast enough to lose steps when probing
    // In the future we need to use a safe speed limit for determining this
    if ((STATE_FLAGS & (1 << STATE_BIT_MOVING)) && (reason != STOP_REASON_PROBE)) {
        STATE_FLAGS |= (1 << STATE_BIT_LOST);
    }
    current_position.stop_reason = reason;

    STATE_FLAGS |=  (1 << STATE_BIT_BUFFER_EMPTY);
    STATE_FLAGS &= ~(1 << STATE_BIT_BUFFER_FULL);
    STATE_FLAGS &= ~(1 << STATE_BIT_MOVING);
    STATE_FLAGS &= ~(1 << STATE_BIT_RUNNING);
    SREG = flg;
}

void movement_start()
{
    TCNT0 = 0;
    TCNT3 = 0;
    TIFR1 = 0;
    TIMSK3 &= (1 << TOIE3);
    TIMSK1 = (1 << TOIE1);
    current_position.stop_reason = STOP_REASON_NONE;
}

int8_t movement_push(struct movement_block * op)
{
    uint8_t new_head = (movement_queue_head + 1) & (MOVEMENT_QUEUE_SIZE - 1);

    if (new_head == movement_queue_tail) {
        STATE_FLAGS |= (1 << STATE_BIT_BUFFER_FULL);
        return -1;
    }

    memcpy(&movement_queue_blocks[movement_queue_head], op, sizeof(struct movement_block));
    movement_queue_head = new_head;
    STATE_FLAGS &= ~(1 << STATE_BIT_BUFFER_EMPTY);
    return 0;
}

ISR(TIMER1_OVF_vect)
{
    if (movement_queue_head == movement_queue_tail) {
        STATE_FLAGS |= (1 << STATE_BIT_BUFFER_EMPTY);
        if (STATE_FLAGS & (1 << STATE_BIT_RUNNING)) {
            movement_stop(STOP_REASON_UNDERRUN);
        }
        return;
    }

    movement_set(&movement_queue_blocks[movement_queue_tail]);
    TIMSK1 |= (1 << TOIE1);
    STATE_FLAGS |= (1 << STATE_BIT_RUNNING);
    movement_queue_tail++;
    if (movement_queue_tail == MOVEMENT_QUEUE_SIZE)
        movement_queue_tail = 0;
    STATE_FLAGS &= ~(1 << STATE_BIT_BUFFER_FULL);
}

static struct movement_block jog_movement;

void movement_jog(struct movement_block * op)
{
    TIMSK1 &= ~(1 << TOIE1);
    if (STATE_FLAGS & (1 << STATE_BIT_MOVING) == 0) {
        TCNT0 = 0;
        TCNT3 = 0;
    }
    STATE_FLAGS &= ~(1 << STATE_BIT_RUNNING);
    memcpy(&jog_movement, op, sizeof(struct movement_block));
    TIMSK3 |= (1 << TOIE3);
}

ISR(TIMER3_OVF_vect)
{
    movement_set(&jog_movement);
}

void movement_set(struct movement_block * next_op)
{
    TIMSK1 = 0;
    current_position.stop_reason = STOP_REASON_NONE;
    intervalX = pgm_read_word(&pulse_timings[abs(next_op->nX)]);
    OCR1A = intervalX;

    intervalY = pgm_read_word(&pulse_timings[abs(next_op->nY)]);
    OCR1B = intervalY;

    intervalZ = pgm_read_word(&pulse_timings[abs(next_op->nZ)]);
    OCR1C = intervalZ;

    uint8_t newport = (STEPPER_PORT & ((1 << STEPPER_PIN_ENABLE)
                                     | (1 << STEPPER_PIN_SPINDLE)))
#if STEPPER_STEP_ACTIVE_LOW
                      | (1 << STEPPER_PIN_STEP_X)
                      | (1 << STEPPER_PIN_STEP_Y)
                      | (1 << STEPPER_PIN_STEP_Z)
#endif
                      ;

    if (next_op->nX != 0) {
        TIMSK1 |= (1 << OCIE1A);
        if (SHOULD_SET_DIR_BIT(next_op->nX))
            newport |= (1 << STEPPER_PIN_DIR_X);
    }
    if (next_op->nY != 0) {
        TIMSK1 |= (1 << OCIE1B);
        if (SHOULD_SET_DIR_BIT(next_op->nY))
            newport |= (1 << STEPPER_PIN_DIR_Y);
    }
    if (next_op->nZ != 0) {
        TIMSK1 |= (1 << OCIE1C);
        if (SHOULD_SET_DIR_BIT(next_op->nZ))
            newport |= (1 << STEPPER_PIN_DIR_Z);
    }

    STEPPER_PORT = newport;
    TIMSK3 = TIMSK1;
    if (TIMSK1 != 0)
        STATE_FLAGS |= (1 << STATE_BIT_MOVING);
    else
        STATE_FLAGS &= ~(1 << STATE_BIT_MOVING);
}

// Take step on X
ISR(TIMER1_COMPA_vect)
{
    OCR1A += intervalX;
    if (DIR_IS_POSITIVE(STEPPER_PIN_DIR_X)) {
        current_position.X++;
    } else {
        current_position.X--;
    }
    ACTIVATE_STEP(STEPPER_PIN_STEP_X);
    OCR3A = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

// Take step on Y
ISR(TIMER1_COMPB_vect)
{
    OCR1B += intervalY;
    if (DIR_IS_POSITIVE(STEPPER_PIN_DIR_Y)) {
        current_position.Y++;
    } else {
        current_position.Y--;
    }
    ACTIVATE_STEP(STEPPER_PIN_STEP_Y);
    OCR3B = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

// Take step on Z
ISR(TIMER1_COMPC_vect)
{
    OCR1C += intervalZ;
    if (DIR_IS_POSITIVE(STEPPER_PIN_DIR_Z)) {
        current_position.Z++;
    } else {
        current_position.Z--;
    }

    ACTIVATE_STEP(STEPPER_PIN_STEP_Z);
    OCR3C = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

void movement_enable_steppers()
{
#if STEPPER_ENABLE_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_ENABLE);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_ENABLE);
#endif
}

void movement_disable_steppers()
{
#if STEPPER_ENABLE_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_ENABLE);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_ENABLE);
#endif
    if (STATE_FLAGS & (1 << STATE_BIT_MOVING)) {
        STATE_FLAGS |= (1 << STATE_BIT_LOST);
    }
}

void movement_enable_spindle()
{
#if SPINDLE_ENABLE_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_SPINDLE);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_SPINDLE);
#endif
}

void movement_disable_spindle()
{
#if SPINDLE_ENABLE_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_SPINDLE);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_SPINDLE);
#endif
}

