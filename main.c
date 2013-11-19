#include <stdint.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "movement.h"
#include "protocol.h"
#include "serial.h"
#include "pins.h"

#ifdef CLIENT_INCLUDE
    #error Something went wrong. Protocol header contains client definitions
#endif

struct status_block status;

static void cmd_send_status()
{
    struct status_block tmp;
    asm volatile("cli");
    memcpy(&tmp, &status, sizeof(struct status_block));
    asm volatile("sei");
    serial_tx(MSG_POSITION);
    serial_tx(STATE_FLAGS);
    serial_tx(0); // GPIO OUTPUTS
    serial_txb(&tmp, sizeof(struct status_block));
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

static void handle_move_queue()
{
    uint8_t countIn = serial_rx();
    uint8_t estop = (STATE_FLAGS & (1 << STATE_BIT_ESTOP)) != 0;

    int i;
    uint8_t countOut = 0;
    uint8_t tag = 0;

    if (!estop) {
        struct movement_block * blk;
        for (i = 0; i < countIn; i++) {
            blk = movement_queue_get_free();
            if (!blk)
                break;
            serial_rxb(blk, sizeof(struct movement_block));
            uint8_t tmp = movement_queue_commit();
            if (countOut == 0)
                tag = tmp; 
            countOut++;
        }
    }
    for (i = countOut; i < countIn; i++) {
        struct movement_block arg;
        serial_rxb(&arg, sizeof(struct movement_block));
    }

    if (estop) {
        serial_tx(RES_ESTOP);
    } else {
        serial_tx(RES_QUEUED);
        serial_tx(countOut);
        serial_tx(tag);
        cmd_send_status();
    }
}

void handle_debug()
{
    serial_tx(RES_NAK);
}

void handle_debug2()
{
    serial_tx(RES_NAK);
}

static int8_t handle_command()
{
    if (!serial_poll())
        return -1;
    uint8_t command = serial_rx();
    uint8_t ESTOP = (STATE_FLAGS & (1 << STATE_BIT_ESTOP)) != 0;
    uint8_t res = ESTOP ? RES_ESTOP : RES_ACK;
    switch (command) {
        case CMD_DEBUG: handle_debug(); return 0;
        case CMD_DEBUG2: handle_debug2(); return 0;

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
            movement_stop(STOP_REASON_ESTOP);
            res = RES_ACK;
            break;

        case CMD_CLEAR_ESTOP:
            if (!pins_is_estop_line_asserted()) {
                STATE_FLAGS &= ~(1 << STATE_BIT_ESTOP);
                res = RES_ACK;
            } else {
                res = RES_NAK;
            }
            break;

        case CMD_GET_STATE:
            cmd_send_status();
            return 0;

        case CMD_MOVE_STOP:
            if (!ESTOP) movement_stop(STOP_REASON_COMMAND);
            break;

        case CMD_MOVE_START:
            if (ESTOP)
                break;
            movement_cycle_start();
            break;

        case CMD_MOTORS_ON:
            if (!ESTOP) pins_enable_steppers();
            break;

        case CMD_MOTORS_OFF:
            pins_disable_steppers();
            res = RES_ACK;
            break;

        case CMD_SPINDLE_ON:
            if (!ESTOP) pins_enable_spindle();
            break;

        case CMD_SPINDLE_OFF:
            pins_disable_spindle();
            res = RES_ACK;
            break;

        case CMD_MOVE_QUEUE:
            handle_move_queue();
            return 0;

        case CMD_MOVE_JOG: {
            struct movement_block arg;
            serial_rxb(&arg, sizeof(struct movement_block));
            if (ESTOP) break;
            if (movement_jog(&arg) < 0)
                res = RES_NAK;
        } break;

        case CMD_SET_ZERO_POS:
            if (ESTOP) break;
            status.X = 0;
            status.Y = 0;
            status.Z = 0;
            break;

        default:
            serial_tx(RES_UNKNOWN);
            return 0;
    }

    serial_tx(res);
    return 0;
}

int main()
{
#ifdef ESTOP_AFTER_RESET
    STATE_FLAGS |= (1 << STATE_BIT_ESTOP);
#endif
    DDRB |= 0x80;

    pins_init();
    movement_init();
    serial_init();

    sei();

    for (;;) {
        handle_command();
        if (SYS_FLAGS_A & (1 << EVENT_TIMESLICE)) {
            SYS_FLAGS_A &= ~(1 << EVENT_TIMESLICE);
            timeslice_elapsed();
        }
    }
}

