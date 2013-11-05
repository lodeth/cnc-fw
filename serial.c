#include <avr/io.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "protocol.h"
#include "config.h"

static uint8_t transmit_queue_head;
volatile static uint8_t transmit_queue_tail;
static uint8_t transmit_queue[SERIAL_TX_QUEUE_SIZE];

volatile static uint8_t receive_queue_head;
static uint8_t receive_queue_tail;
static uint8_t receive_queue[SERIAL_RX_QUEUE_SIZE];

void serial_init()
{
    // Serial setup to 115200 baud 8N1
    SERIAL_UBRR = 16;
    SERIAL_UCSRnA = (1 << SERIAL_U2X);
    SERIAL_UCSRnB = (1 << SERIAL_RXEN) | (1 << SERIAL_TXEN) | (1 << SERIAL_RXCIE); 
}

ISR(SERIAL_RX_vect)
{
    uint8_t byte = SERIAL_UDR;
    uint8_t new_head = (receive_queue_head + 1) & (SERIAL_RX_QUEUE_SIZE - 1);
    if (new_head == receive_queue_tail) {
        FIRMWARE_PANIC();
    }
    receive_queue[receive_queue_head] = byte;
    receive_queue_head = new_head;
}

uint8_t serial_poll()
{
    if (receive_queue_head == receive_queue_tail)
        return 0;
    else
        return 1;
}

uint8_t serial_rx()
{
    while (receive_queue_head == receive_queue_tail) {
        uint8_t flg = SREG;
        sei();
        asm volatile("sleep");
        SREG = flg;
    }
    uint8_t res = receive_queue[receive_queue_tail];
    receive_queue_tail++;
    if (receive_queue_tail == SERIAL_RX_QUEUE_SIZE)
        receive_queue_tail = 0;
    return res;
}

void serial_rxb(void * byte, uint8_t len)
{
    uint8_t * p = (uint8_t *) byte;
    while (len--) {
        *p = serial_rx();
        p++;
    }
}

void serial_txb(void * byte, uint8_t len)
{
    uint8_t * p = (uint8_t *) byte;
    while (len--) {
        serial_tx(*p);
        p++;
    }
}

void serial_tx(uint8_t byte)
{
    uint8_t new_head = (transmit_queue_head + 1) & (SERIAL_TX_QUEUE_SIZE - 1);

    // Enable interrupts while waiting for space to free up
    if (new_head == transmit_queue_tail) {
        uint8_t flg = SREG;
        sei();
        while (new_head == transmit_queue_tail) { asm volatile("sleep"); }
        SREG = flg;
    }

    transmit_queue[transmit_queue_head] = byte;
    transmit_queue_head = new_head;
    SERIAL_UCSRnB |= (1 << SERIAL_UDRIE);
}

ISR(SERIAL_UDRE_vect)
{
    if (transmit_queue_head == transmit_queue_tail) {
        return;
    }

    SERIAL_UDR = transmit_queue[transmit_queue_tail];
    transmit_queue_tail++;
    if (transmit_queue_tail == SERIAL_TX_QUEUE_SIZE)
        transmit_queue_tail = 0;
}

