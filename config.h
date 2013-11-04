#ifndef CONFIG_H
#define CONFIG_H

// -----------------------------------------------------------------
//
//  General configuration for magic values
//
// -----------------------------------------------------------------

// This is an experimentally determined value for _my_ stepper drivers  (5 < value < 100) 
#define STEP_PULSE_WIDTH_TICKS      10

// Use USART1 instead of USART0. Comment to disable.
#define USE_USART1                  1

// Please keep this a power of two or hilarity ensues (value <= 256)
#define SERIAL_TX_QUEUE_SIZE        64
#define SERIAL_RX_BUFFER_SIZE       32

// Please keep this a power of two or hilarity ensues (value <= 256)
#define MOVEMENT_QUEUE_SIZE         256

// Count of TIMER0 overflows before sending status (value <= 256)
#define STATUS_PUSH_INTERVAL        32 

// -----------------------------------------------------------------
//
//  Configuration for inputs
//
// -----------------------------------------------------------------
// Default values result in the following:
//    LIMIT_X  = PK.0
//    LIMIT_Y  = PK.1
//    LIMIT_Z  = PK.2
//    PROBE    = PK.3

// BOTH asserting or deasserting a limit or probe signal stop the
// machine. This behaviour makes homing easier.

// Pull-ups are enabled, so whether you use normally closed switches
// or not, you should wire them to the ground. I'd recommend that
// you use them so a wire failure will trip the limit.

#define INPUT_PORT          PORTK
#define INPUT_DDR           DDRK
#define INPUT_PIN           PINK
#define INPUT_PCIE          PCIE2
#define INPUT_PCIMSK        PCMSK2
#define INPUT_PCI_vect      PCINT2_vect

#define INPUT_LIMIT_X       0
#define INPUT_LIMIT_Y       1
#define INPUT_LIMIT_Z       2
#define INPUT_PROBE         3

// -----------------------------------------------------------------
//
//  Configuration for motor outputs
//
// -----------------------------------------------------------------
//
// Default values result in the following:
//    STEP_X  = PF.0
//    STEP_Y  = PF.1
//    STEP_Z  = PF.2
//    DIR_X   = PF.3
//    DIR_Y   = PF.4
//    DIR_Z   = PF.5
//    ENABLE  = PF.6
//    SPINDLE = PF.7

#define STEPPER_PORT            PORTF
#define STEPPER_DDR             DDRF

// Signal polarity
#define STEPPER_ENABLE_ACTIVE_LOW  1
#define STEPPER_SPINDLE_ACTIVE_LOW 1
#define STEPPER_STEP_ACTIVE_LOW    1
#define STEPPER_DIR_POSITIVE_LOW   1

// Signal port pins
#define STEPPER_PIN_STEP_X          0
#define STEPPER_PIN_STEP_Y          1
#define STEPPER_PIN_STEP_Z          2
#define STEPPER_PIN_DIR_X           3
#define STEPPER_PIN_DIR_Y           4
#define STEPPER_PIN_DIR_Z           5
#define STEPPER_PIN_ENABLE          6
#define STEPPER_PIN_SPINDLE         7

// -----------------------------------------------------------------
//
//  Definitions for USARTs follow
//
// -----------------------------------------------------------------

#if USE_USART1
    #define SERIAL_UDRE_vect        USART1_UDRE_vect
    #define SERIAL_RX_vect          USART1_RX_vect
    #define SERIAL_UDR              UDR1
    #define SERIAL_UBRR             UBRR1
    #define SERIAL_UCSRnA           UCSR1A
    #define SERIAL_UCSRnB           UCSR1B
    // These should be equal for all serial interfaces, but let's define them anyway
    #define SERIAL_U2X              U2X1
    #define SERIAL_RXEN             RXEN1
    #define SERIAL_TXEN             TXEN1
    #define SERIAL_RXCIE            RXCIE1
    #define SERIAL_UDRIE            UDRIE1
#else
    #define SERIAL_UDRE_vect        USART0_UDRE_vect
    #define SERIAL_RX_vect          USART0_RX_vect
    #define SERIAL_UDR              UDR0
    #define SERIAL_UBRR             UBRR0
    #define SERIAL_UCSRnA           UCSR0A
    #define SERIAL_UCSRnB           UCSR0B
    #define SERIAL_U2X              U2X0
    #define SERIAL_RXEN             RXEN0
    #define SERIAL_TXEN             TXEN0
    #define SERIAL_RXCIE            RXCIE0
    #define SERIAL_UDRIE            UDRIE0
#endif

#endif

