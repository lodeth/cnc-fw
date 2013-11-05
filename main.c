#include <stdint.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "movement.h"
#include "protocol.h"
#include "serial.h"

struct position_block current_position;

void timeslice_elapsed()
{
    static uint8_t status_push_cnt;
    if (++status_push_cnt == STATUS_PUSH_INTERVAL) {
        status_push_cnt = 0;
        serial_tx(MSG_POSITION);
        serial_tx(STATE_FLAGS);

        // Should not need to disable interrupts, since this should be triggered
        // right after the only code that can modify these values
        serial_txb(&current_position, sizeof(struct position_block));
    }
}

int8_t handle_command()
{
    if (!serial_poll())
        return -1;
    uint8_t command = serial_rx();
    if (STATE_FLAGS & (1 << STATE_BIT_ESTOP)) {
        switch (command) {
            case CMD_NOP:
                return 0;

            case CMD_SET_ESTOP:
            case CMD_ECHO:
                serial_tx(RES_ACK);
                return 0;

            case CMD_CLEAR_ESTOP:
                if ((INPUT_PORT & (1 << INPUT_ESTOP))
#ifdef INPUT_ESTOP_ACTIVE_LOW
    !=
#else
    ==
#endif
                0) {
                    STATE_FLAGS &= ~(1 << STATE_BIT_ESTOP);
                    serial_tx(RES_ACK);
                    return 0;
                }
                break;
            default:
                break;
        }
        serial_tx(RES_ESTOP);
        return 0;
    }

    static uint8_t expected_queue_seqno;
    switch (command) {
        case CMD_NOP:
            // Special case, no response at all
            return 0;

        case CMD_ECHO:
            break;

        case CMD_CLEAR_ESTOP:
            break;

        case CMD_SET_ESTOP:
            expected_queue_seqno = 0;
            movement_stop(STOP_REASON_ESTOP);
            break;

        case CMD_MOVE_STOP:
            expected_queue_seqno = 0;
            movement_stop(STOP_REASON_COMMAND);
            break;

        case CMD_MOVE_START:
            movement_start();
            break;

        case CMD_MOTORS_ON:
            movement_enable_steppers();
            break;

        case CMD_MOTORS_OFF:
            movement_disable_steppers();
            break;

        case CMD_SPINDLE_ON:
            movement_enable_spindle();
            break;

        case CMD_SPINDLE_OFF:
            movement_disable_spindle();
            break;

        case CMD_MOVE_QUEUE: {
            uint8_t seqno = serial_rx();
            struct movement_block arg;
            serial_rxb(&arg, sizeof(struct movement_block));
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
        } return 0;

        case CMD_MOVE_JOG: {
            struct movement_block arg;
            serial_rxb(&arg, sizeof(struct movement_block));
            if (movement_jog(&arg) < 0)
                serial_tx(RES_NAK);
            else
                serial_tx(RES_ACK);
        } return 0;

        case CMD_ZERO_POSITION:
            current_position.X = 0;
            current_position.Y = 0;
            current_position.Z = 0;
            STATE_FLAGS &= ~(1 << STATE_BIT_LOST);
            break;

        default:
            serial_tx(RES_NAK);
            return 0;
    }
    serial_tx(RES_ACK);
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

    DDRD = 0x03;
    PORTD = 0x01;

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
        // Wait for an interrupt and check if any flags were set
        asm volatile("sleep");
        for (;;) {
            if (EVENT_FLAGS & (1 << EVENT_TIMESLICE)) {
                EVENT_FLAGS &= ~(1 << EVENT_TIMESLICE);
                timeslice_elapsed();
            }
            if (handle_command())
                break;
        }
    }
}

