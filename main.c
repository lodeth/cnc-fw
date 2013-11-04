#include <stdint.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "movement.h"
#include "protocol.h"
#include "serial.h"

struct position_block current_position;

static uint8_t expected_queue_seqno;
uint8_t protocol_handle_first_byte(uint8_t byte)
{
    // Was first byte of packet,
    // handle single byte commands directly or calc expected len

    switch (byte) {
        case CMD_NOP: return 0; // Special case, no response at all
        case CMD_ECHO: break;

        case CMD_MOVE_STOP:     expected_queue_seqno = 0; movement_stop(); break;
        case CMD_MOVE_START:    movement_start(); break;

        case CMD_MOTORS_ON:     movement_enable_steppers(); break;
        case CMD_MOTORS_OFF:    movement_disable_steppers(); break;
        case CMD_SPINDLE_ON:    movement_enable_spindle(); break;
        case CMD_SPINDLE_OFF:   movement_disable_spindle(); break;

        case CMD_MOVE_QUEUE:    return 1 + sizeof(struct movement_block);
        case CMD_MOVE_JOG:      return sizeof(struct movement_block);

        case CMD_ZERO_POSITION:
            current_position.pX = 0;
            current_position.pY = 0;
            current_position.pZ = 0;
            current_position.state &= ~(1 << STATE_BIT_LOST);
            break;

        default:
            serial_tx(RES_NAK);
            break;
    }
    serial_tx(RES_ACK);
    return 0;
}

void protocol_handle_long_command(uint8_t * buffer)
{

    uint8_t byte = buffer[0];

    if (byte == CMD_MOVE_JOG) {
        movement_jog((struct movement_block *) (buffer + 1));
        serial_tx(RES_ACK);

    } else if (byte == CMD_MOVE_QUEUE) {
        uint8_t seqno = buffer[1];
        if (seqno != expected_queue_seqno) {
            serial_tx(ERR_SEQUENCE);
            serial_tx(expected_queue_seqno);
        } else {
            int8_t res = movement_push((struct movement_block *) (buffer + 2));
            if (res < 0) {
                serial_tx(ERR_FULL);
                serial_tx(expected_queue_seqno);
            } else {
                expected_queue_seqno = seqno + 1;
                serial_tx(RES_QUEUED);
                serial_tx(expected_queue_seqno);
            }
        }
    }
}

ISR(TIMER0_OVF_vect)
{
    static uint8_t cnt;
    if (++cnt != STATUS_PUSH_INTERVAL)
        return;
    cnt = 0;
    serial_tx(MSG_POSITION);
    serial_txb(&current_position, sizeof(struct position_block));
}


ISR(INPUT_PCI_vect)
{
    uint8_t change = INPUT_PIN ^ current_position.inputs;
    current_position.inputs = INPUT_PIN;
    if (change & ((1 << INPUT_LIMIT_X) | (1 << INPUT_LIMIT_Y) | (1 << INPUT_LIMIT_Z))) {
        serial_tx(MSG_LIMIT_TRIPPED);
        movement_stop();
    }

    if (change & (1 << INPUT_PROBE)) {
        serial_tx(MSG_PROBE_TRIPPED);
        // We preserve STATE_BIT_LOST across the stop.
        // It should be safe to assume that we're not moving fast enough to lose steps when probing
        // In the future we need to use a safe speed limit for determining this
        uint8_t state = current_position.state;
        movement_stop();
        if (state & (1 << STATE_BIT_LOST) == 0)
            current_position.state &= ~(1 << STATE_BIT_LOST);

    }
}

int main()
{
    movement_init();
    serial_init();

    INPUT_DDR  = 0x00;
    INPUT_PORT = 0xff;

    DDRD = 0x03;
    PORTD = 0x01;

    sei();

    //TCCR0A = 0;
    //TCCR0B = (5<<CS00);
    //TIMSK0 = (1<<TOIE0);

    PCICR |= (1 << INPUT_PCIE);
    INPUT_PCIMSK = 0xff;

    serial_tx(MSG_RESET);

    for (;;) { asm volatile("sleep"); }
}

