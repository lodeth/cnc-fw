#include <avr/io.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "protocol.h"
#include "config.h"

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
    return ((SERIAL_UCSRnA & (1 << SERIAL_RXCn)) != 0);
}

uint8_t serial_rx()
{
    while ((SERIAL_UCSRnA & (1 << SERIAL_RXCn)) == 0) { };
    return SERIAL_UDR;
}

void serial_tx(uint8_t byte)
{
    while ((SERIAL_UCSRnA & (1 << SERIAL_UDREn)) == 0) {};
    SERIAL_UDR = byte;
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



// These ISRs are used to wake the device up from sleep
ISR(SERIAL_UDRE_vect) { }
ISR(SERIAL_RX_vect) { }

