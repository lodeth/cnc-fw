#include <stdint.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "movement.h"
#include "protocol.h"
#include "serial.h"

#ifdef CLIENT_INCLUDE
    #error Something went wrong. Protocol header contains client definitions
#endif

struct position_block current_position;

static void send_status()
{
    serial_tx(MSG_POSITION);
    serial_tx(STATE_FLAGS);
    // Should not need to disable interrupts, since this should be triggered
    // right after the only code that can modify these values
    serial_txb(&current_position, sizeof(struct position_block));
}

void timeslice_elapsed()
{
    uint16_t blink_speed;
    if (STATE_FLAGS & (1 << STATE_BIT_ESTOP)) 
        blink_speed = 0x01;
    else if (STATE_FLAGS & (1 << STATE_BIT_RUNNING))
        blink_speed = 0x10;
    else
        blink_speed = 0x30;

#if MOVEMENT_TIMESLICE == TIMESLICE_4MS
    blink_speed *= 8;
#endif

    static uint16_t blinker = 0;
    if (blinker++ > blink_speed) {
        PORTB ^= 0x80;
        blinker = 0;
    }

#ifdef STATUS_PUSH_INTERVAL
    static uint8_t status_push_cnt;
    if (++status_push_cnt == STATUS_PUSH_INTERVAL) {
        status_push_cnt = 0;
    }
#endif
}

int8_t handle_command()
{
    if (!serial_poll())
        return -1;
    uint8_t command = serial_rx();

    uint8_t ESTOP = (STATE_FLAGS & (1 << STATE_BIT_ESTOP)) != 0;

    uint8_t res = ESTOP ? RES_ESTOP : RES_ACK;

    static uint8_t expected_queue_seqno;
    switch (command) {
        case CMD_NOP:
            // Special case, no response at all
            return 0;
        case CMD_PING1:
            res = RES_PING1;
            break;
        case CMD_PING2:
            res = RES_PING2;
            break;

        case CMD_SET_ESTOP:
            expected_queue_seqno = 0;
            movement_stop(STOP_REASON_ESTOP);
            res = RES_ACK;
            break;

        case CMD_CLEAR_ESTOP:
            #ifdef INPUT_ESTOP_ACTIVE_LOW
            if ((INPUT_PORT & (1 << INPUT_ESTOP)) != 0)
            #else
            if ((INPUT_PORT & (1 << INPUT_ESTOP)) == 0)
            #endif
            {
                STATE_FLAGS &= ~(1 << STATE_BIT_ESTOP);
                res = RES_ACK;
            } else {
                res = RES_NAK;
            }
            break;

        case CMD_GET_STATE:
            send_status();
            return 0;

        case CMD_MOVE_STOP:
            expected_queue_seqno = 0;
            if (!ESTOP) movement_stop(STOP_REASON_COMMAND);
            break;

        case CMD_MOVE_START:
            if (!ESTOP) movement_start();
            break;

        case CMD_MOTORS_ON:
            if (!ESTOP) movement_enable_steppers();
            break;

        case CMD_MOTORS_OFF:
            if (!ESTOP) movement_disable_steppers();
            break;

        case CMD_SPINDLE_ON:
            if (!ESTOP) movement_enable_spindle();
            break;

        case CMD_SPINDLE_OFF:
            if (!ESTOP) movement_disable_spindle();
            break;

        case CMD_MOVE_QUEUE: {
            uint8_t seqno = serial_rx();
            struct movement_block arg;
            serial_rxb(&arg, sizeof(struct movement_block));
            if (ESTOP) break;

            if (seqno != expected_queue_seqno) {
                serial_tx(ERR_SEQUENCE);
            } else {
                int8_t res = movement_push(&arg);
                if (res < 0) {
                    serial_tx(ERR_FULL);
                } else {
                    expected_queue_seqno = seqno + 1;
                    serial_tx(RES_QUEUED);
                }
            }
            serial_tx(expected_queue_seqno);
            return 0;
        }

        case CMD_MOVE_JOG: {
            struct movement_block arg;
            serial_rxb(&arg, sizeof(struct movement_block));
            if (ESTOP) break;

            if (movement_jog(&arg) < 0)
                res = RES_NAK;
        } break;

        case CMD_SET_ZERO_POS:
            if (ESTOP) break;

            current_position.X = 0;
            current_position.Y = 0;
            current_position.Z = 0;
            STATE_FLAGS &= ~(1 << STATE_BIT_LOST);
            break;

        default:
            serial_tx(RES_UNKNOWN);
            return 0;
    }
    serial_tx(res);
    return 0;
}

ISR(INPUT_PCI_vect)
{
    uint8_t change = INPUT_PIN ^ current_position.inputs;
    current_position.inputs = INPUT_PIN;

    if ((INPUT_PIN & (1 << INPUT_ESTOP))
#ifdef INPUT_ESTOP_ACTIVE_LOW
    ==
#else
    !=
#endif
    0) { movement_stop(STOP_REASON_ESTOP); }

    if (change & ((1 << INPUT_LIMIT_X) | (1 << INPUT_LIMIT_Y) | (1 << INPUT_LIMIT_Z))) {
        movement_stop(STOP_REASON_LIMIT);
    } else if (change & (1 << INPUT_PROBE))
        movement_stop(STOP_REASON_PROBE);
}

int main()
{
    movement_init();
    serial_init();

    INPUT_DDR  = 0x00;
    INPUT_PORT = 0xff;
    current_position.inputs = INPUT_PIN;

    DDRB = 0x80;

    sei();

    PCICR |= (1 << INPUT_PCIE);
    INPUT_PCIMSK = 0xff;

    if ((INPUT_PIN & (1 << INPUT_ESTOP))
#ifdef INPUT_ESTOP_ACTIVE_LOW
    ==
#else
    !=
#endif
    0) { movement_stop(STOP_REASON_ESTOP); }

    serial_tx(MSG_RESET);
    for (;;) {
        if (!handle_command()) {
            asm volatile("sleep");
        }
        if (EVENT_FLAGS & (1 << EVENT_TIMESLICE)) {
            EVENT_FLAGS &= ~(1 << EVENT_TIMESLICE);
            timeslice_elapsed();
        }
    }
}

