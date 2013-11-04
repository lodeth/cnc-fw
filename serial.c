#include <avr/io.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "protocol.h"
#include "config.h"

uint8_t protocol_handle_first_byte(uint8_t byte);
void protocol_handle_long_command(uint8_t * buffer);

static uint8_t transmit_queue_head;
volatile static uint8_t transmit_queue_tail;
static uint8_t transmit_queue[SERIAL_TX_QUEUE_SIZE];

void serial_init()
{
    // Serial setup to 115200 baud 8N1
    SERIAL_UBRR = 16;
    SERIAL_UCSRnA = (1 << SERIAL_U2X);
    SERIAL_UCSRnB = (1 << SERIAL_RXEN) | (1 << SERIAL_TXEN) | (1 << SERIAL_RXCIE); 
}

static uint8_t rx_count = 0;
static uint8_t rx_buffer[SERIAL_RX_BUFFER_SIZE];
static uint8_t expected_len;

ISR(SERIAL_RX_vect)
{
    uint8_t byte = SERIAL_UDR;
    if (rx_count == 0) {
        expected_len = protocol_handle_first_byte(byte);
        if (expected_len == 0)
            return;
        if (expected_len >= SERIAL_RX_BUFFER_SIZE) {
            FIRMWARE_PANIC();
        }
    }

    rx_buffer[rx_count++] = byte;
    if (rx_count <= expected_len)
        return;
    rx_count = 0;
    protocol_handle_long_command(rx_buffer);
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

