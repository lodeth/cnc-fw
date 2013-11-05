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

#ifdef STEPPER_DIR_POSITIVE_LOW
#define DIR_IS_POSITIVE(d) ((STEPPER_PORT & (1 << d)) == 0)
#define SHOULD_SET_DIR_BIT(d) (d) < 0
#else
#define DIR_IS_POSITIVE(d) (STEPPER_PORT & (1 << d))
#define SHOULD_SET_DIR_BIT(d) (d) > 0
#endif

#ifdef STEPPER_STEP_ACTIVE_LOW
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

static void movement_set(struct movement_block * next_op);
static void flush_steps_taken();

void movement_init()
{
    movement_stop(STOP_REASON_RESET);
#ifdef ESTOP_AFTER_RESET
    STATE_FLAGS |= (1 << STATE_BIT_ESTOP);
#endif
    STEPPER_DDR = 0xff;

    movement_disable_steppers();
    movement_disable_spindle();

#ifdef STEPPER_STEP_ACTIVE_LOW
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
    TCCR3B = (MOVEMENT_TIMESLICE << CS30);
}

void movement_stop(uint8_t reason)
{
    uint8_t flg = SREG;
    cli();
    TIMSK1 = 0;
    TIMSK3 = (1 << TOIE3);

    movement_queue_head = 0;
    movement_queue_tail = 0;

    flush_steps_taken();

    current_position.stop_reason = reason;

    // It should be safe to assume that we're not moving fast enough to lose steps when probing
    // In the future we need to use a safe speed limit for determining this
    if (reason != STOP_REASON_PROBE) {
        if (STATE_FLAGS & (1 << STATE_BIT_MOVING))
            STATE_FLAGS |= (1 << STATE_BIT_LOST);
        if (reason == STOP_REASON_ESTOP) {
            STATE_FLAGS |= STATE_BIT_ESTOP;
            movement_disable_steppers();
            movement_disable_spindle();
        }
    }

    STATE_FLAGS |=  (1 << STATE_BIT_BUFFER_EMPTY);
    STATE_FLAGS &= ~(1 << STATE_BIT_BUFFER_FULL);
    STATE_FLAGS &= ~(1 << STATE_BIT_MOVING);
    STATE_FLAGS &= ~(1 << STATE_BIT_RUNNING);
    STATE_FLAGS &= ~(1 << STATE_BIT_JOGGING);
    SREG = flg;
}

void movement_start()
{
    TCNT0 = 0;
    TCNT3 = 0;
    TIFR1 = 0;
    TIMSK3 &= ~(1 << TOIE3);
    STATE_FLAGS &= ~(1 << STATE_BIT_JOGGING);

    flush_steps_taken();
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
    flush_steps_taken();
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
    EVENT_FLAGS |= (1 << EVENT_TIMESLICE);
}

static struct movement_block jog_movement;
static struct movement_block jog_target_speed;

int8_t movement_jog(struct movement_block * op)
{
    if (STATE_FLAGS & (1 << STATE_BIT_RUNNING))
        return -1;

    if ((STATE_FLAGS & (1 << STATE_BIT_MOVING)) == 0) {
        TCNT0 = 0;
        TCNT3 = 0;
        memset(&jog_movement, 0, sizeof(struct movement_block));
    }
    memcpy(&jog_target_speed, op, sizeof(struct movement_block));
    flush_steps_taken();
    STATE_FLAGS |= (1 << STATE_BIT_JOGGING);
    return 0;
}

ISR(TIMER3_OVF_vect)
{
    if ((STATE_FLAGS & (1 << STATE_BIT_RUNNING)) == 0)
        EVENT_FLAGS |= (1 << EVENT_TIMESLICE);

    if ((STATE_FLAGS & (1 << STATE_BIT_JOGGING)) == 0)
        return;

    flush_steps_taken();
    int8_t d;

    // Slide closer to target speed
    d = jog_target_speed.X - jog_movement.X;
    if (d < 0)
        jog_movement.X--;
    else if (d > 0)
        jog_movement.X++;
    d = jog_target_speed.Y - jog_movement.Y;
    if (d < 0)
        jog_movement.Y--;
    else if (d > 0)
        jog_movement.Y++;
    d = jog_target_speed.Z - jog_movement.Z;
    if (d < 0)
        jog_movement.Z--;
    else if (d > 0)
        jog_movement.Z++;

    movement_set(&jog_movement);
    TIMSK3 |= (1 << TOIE3);
}

static void movement_set(struct movement_block * next_op)
{
    TIMSK1 = 0;
    current_position.stop_reason = STOP_REASON_NONE;
    intervalX = pgm_read_word(&pulse_timings[abs(next_op->X)]);
    OCR1A = intervalX;

    intervalY = pgm_read_word(&pulse_timings[abs(next_op->Y)]);
    OCR1B = intervalY;

    intervalZ = pgm_read_word(&pulse_timings[abs(next_op->Z)]);
    OCR1C = intervalZ;

    uint8_t newport = (STEPPER_PORT & ((1 << STEPPER_PIN_ENABLE)
                                     | (1 << STEPPER_PIN_SPINDLE)))
#ifdef STEPPER_STEP_ACTIVE_LOW
                      | (1 << STEPPER_PIN_STEP_X)
                      | (1 << STEPPER_PIN_STEP_Y)
                      | (1 << STEPPER_PIN_STEP_Z)
#endif
                      ;

    if (next_op->X != 0) {
        TIMSK1 |= (1 << OCIE1A);
        if (SHOULD_SET_DIR_BIT(next_op->X))
            newport |= (1 << STEPPER_PIN_DIR_X);
    }
    if (next_op->Y != 0) {
        TIMSK1 |= (1 << OCIE1B);
        if (SHOULD_SET_DIR_BIT(next_op->Y))
            newport |= (1 << STEPPER_PIN_DIR_Y);
    }
    if (next_op->Z != 0) {
        TIMSK1 |= (1 << OCIE1C);
        if (SHOULD_SET_DIR_BIT(next_op->Z))
            newport |= (1 << STEPPER_PIN_DIR_Z);
    }

    STEPPER_PORT = newport;
    TIMSK3 = TIMSK1;
    if (TIMSK1 != 0)
        STATE_FLAGS |= (1 << STATE_BIT_MOVING);
    else
        STATE_FLAGS &= ~(1 << STATE_BIT_MOVING);
}

// Count for steps actually taken
static uint8_t steps_taken_X = 0;
static uint8_t steps_taken_Y = 0;
static uint8_t steps_taken_Z = 0;

static void flush_steps_taken()
{
    if (DIR_IS_POSITIVE(STEPPER_PIN_DIR_X)) {
        current_position.X += steps_taken_X;
    } else {
        current_position.X -= steps_taken_X;
    }
    steps_taken_X = 0;

    if (DIR_IS_POSITIVE(STEPPER_PIN_DIR_Y)) {
        current_position.Y += steps_taken_Y;
    } else {
        current_position.Y -= steps_taken_Y;
    }
    steps_taken_Y = 0;

    if (DIR_IS_POSITIVE(STEPPER_PIN_DIR_Z)) {
        current_position.Z += steps_taken_Z;
    } else {
        current_position.Z -= steps_taken_Z;
    }
    steps_taken_Z = 0;
}

// Take step on X
ISR(TIMER1_COMPA_vect)
{
    OCR1A += intervalX;
    steps_taken_X++;
    ACTIVATE_STEP(STEPPER_PIN_STEP_X);
    OCR3A = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

// Take step on Y
ISR(TIMER1_COMPB_vect)
{
    OCR1B += intervalY;
    steps_taken_Y++;

    ACTIVATE_STEP(STEPPER_PIN_STEP_Y);
    OCR3B = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

// Take step on Z
ISR(TIMER1_COMPC_vect)
{
    OCR1C += intervalZ;
    steps_taken_Z++;
    ACTIVATE_STEP(STEPPER_PIN_STEP_Z);
    OCR3C = TCNT3 + STEP_PULSE_WIDTH_TICKS;
}

void movement_enable_steppers()
{
#ifdef STEPPER_ENABLE_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_ENABLE);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_ENABLE);
#endif
}

void movement_disable_steppers()
{
#ifdef STEPPER_ENABLE_ACTIVE_LOW
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
#ifdef SPINDLE_ENABLE_ACTIVE_LOW
    STEPPER_PORT &= ~(1 << STEPPER_PIN_SPINDLE);
#else
    STEPPER_PORT |= (1 << STEPPER_PIN_SPINDLE);
#endif
}

void movement_disable_spindle()
{
#ifdef SPINDLE_ENABLE_ACTIVE_LOW
    STEPPER_PORT |= (1 << STEPPER_PIN_SPINDLE);
#else
    STEPPER_PORT &= ~(1 << STEPPER_PIN_SPINDLE);
#endif
}

