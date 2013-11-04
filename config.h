#ifndef CONFIG_H
#define CONFIG_H

// This is an experimentally determined value for _my_ stepper drivers
#define STEP_PULSE_WIDTH_TICKS 10

// Use USART1 instead of USART0
#define USE_USART1  1

// Please keep this a power of two or hilarity ensues
#define SERIAL_TX_QUEUE_SIZE    64
#define SERIAL_RX_BUFFER_SIZE   32

// Please keep this a power of two or hilarity ensues
#define MOVEMENT_QUEUE_SIZE 256

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
// Please do not touch these unless it is to change axis order
// There are still hardcoded values left in the code
// It is assumed that:
//    STEP_BIT_n == AXIS_BIT_n
//    DIR_BIT_n == AXIS_BIT_n + 3
//
// Default values result in the following:
//    /STEP_X  = PF.0
//    /STEP_Y  = PF.1
//    /STEP_Z  = PF.2
//    /DIR_X   = PF.3
//    /DIR_Y   = PF.4
//    /DIR_Z   = PF.5
//    /ENABLE  = PF.6
//    /SPINDLE = PF.7
// Do note that all signals are ACTIVE LOW

#define STEPPER_PORT        PORTF
#define STEPPER_DDR         DDRF
#define AXIS_BIT_X          0
#define AXIS_BIT_Y          1
#define AXIS_BIT_Z          2

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
