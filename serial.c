#include <avr/io.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "protocol.h"
#include "config.h"


/*
static uint8_t transmit_queue_head;
volatile static uint8_t transmit_queue_tail;
static uint8_t transmit_queue[SERIAL_TX_QUEUE_SIZE];
volatile static uint8_t receive_queue_head;
static uint8_t receive_queue_tail;
static uint8_t receive_queue[SERIAL_RX_QUEUE_SIZE];
*/

void serial_init()
{
    // USART0 for debug atm
    UCSR0B = (1 << SERIAL_RXEN) | (1 << SERIAL_TXEN);
    UCSR0A = (1 << SERIAL_U2X);
    UBRR0 = 16;

    // Serial setup to 115200 baud 8N1
    SERIAL_UCSRnB = (1 << SERIAL_RXEN) | (1 << SERIAL_TXEN);
    SERIAL_UCSRnA = (1 << SERIAL_U2X);
    SERIAL_UBRR = 16;

    // Enable interrupts right away
    SERIAL_UCSRnB |= (1 << SERIAL_RXCIE); 
    SERIAL_UCSRnB |= (1 << SERIAL_UDRIE);
}

uint8_t serial_poll()
{
    return (SERIAL_UCSRnA & (1 << SERIAL_RXCn)) != 0;
    /*
    if (receive_queue_head == receive_queue_tail)
        return 0;
    else
        return 1;
    */
}

uint8_t serial_rx()
{
    while ((SERIAL_UCSRnA & (1 << SERIAL_RXCn)) == 0) { };
    return SERIAL_UDR;
    /*
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
    */
}

void serial_tx(uint8_t byte)
{
    while ((SERIAL_UCSRnA & (1 << SERIAL_UDREn)) == 0) {};
    SERIAL_UDR = byte;
/*
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
    */
}

// These ISRs are used to wake the device up from sleep
ISR(SERIAL_UDRE_vect)
{
/*
    if (transmit_queue_head == transmit_queue_tail) {
        return;
    }

    SERIAL_UDR = transmit_queue[transmit_queue_tail];
    transmit_queue_tail++;
    if (transmit_queue_tail == SERIAL_TX_QUEUE_SIZE)
        transmit_queue_tail = 0;
*/
}

ISR(SERIAL_RX_vect)
{
    /*
    uint8_t byte = SERIAL_UDR;
    uint8_t new_head = (receive_queue_head + 1) & (SERIAL_RX_QUEUE_SIZE - 1);
    if (new_head == receive_queue_tail) {
        FIRMWARE_PANIC();
    }
    receive_queue[receive_queue_head] = byte;
    receive_queue_head = new_head;
    */
}

// Helper functions for buffers
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
