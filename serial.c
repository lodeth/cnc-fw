#include <avr/io.h>
#include <avr/interrupt.h>

#include "serial.h"
#include "protocol.h"

uint8_t protocol_handle_first_byte(uint8_t byte);
void protocol_handle_long_command(uint8_t * buffer);

// Keep this a power of two or suffer
#define TRANSMIT_QUEUE_SIZE 64
static uint8_t transmit_queue_head;
volatile static uint8_t transmit_queue_tail;
static uint8_t transmit_queue[TRANSMIT_QUEUE_SIZE];

void serial_init()
{
    // Serial setup to 115200 baud 8N1
    UBRR1 = 16;
    UCSR1A = (1 << U2X1);
    UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1); 
}

#define RX_BUFFER_SIZE 32
static uint8_t rx_count = 0;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t expected_len;

ISR(USART1_RX_vect)
{
    uint8_t byte = UDR1;
    if (rx_count == 0) {
        expected_len = protocol_handle_first_byte(byte);
        if (expected_len == 0)
            return;
        if (expected_len >= RX_BUFFER_SIZE) {
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
    // Disable interrupts while pushing data to TX queue
    uint8_t flg = SREG;
    cli();
    uint8_t * p = (uint8_t *) byte;
    while (len--) {
        serial_tx(*p);
        p++;
    }
    SREG = flg;
}

void serial_tx(uint8_t byte)
{
    uint8_t new_head = (transmit_queue_head + 1) & (TRANSMIT_QUEUE_SIZE - 1);

    // Enable interrupts while waiting for space to free up
    if (new_head == transmit_queue_tail) {
        uint8_t flg = SREG;
        sei();
        while (new_head == transmit_queue_tail) { asm volatile("sleep"); }
        SREG = flg;
    }

    transmit_queue[transmit_queue_head] = byte;
    transmit_queue_head = new_head;
    UCSR1B |= (1<<UDRIE1); // Force triggering of interrupt if transmitter was idle
}

ISR(USART1_UDRE_vect)
{
    if (transmit_queue_head == transmit_queue_tail)
        return;
    UDR1 = transmit_queue[transmit_queue_tail];
    transmit_queue_tail++;
    if (transmit_queue_tail == TRANSMIT_QUEUE_SIZE)
        transmit_queue_tail = 0;
}

