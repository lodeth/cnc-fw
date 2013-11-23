#ifndef CONFIG_H
#define CONFIG_H

// -----------------------------------------------------------------
//
//  General configuration for magic values
//
// -----------------------------------------------------------------

// Timeslice of motion. Selects base value for the speeds
// Note that this affects the pulse width tick value also
        // CPU clock divisor 1
        // Actual timeslice of motion 4.096 ms
        // Pulse width base 62.5 ns
        #define TIMESLICE_4MS   1
        // CPU clock divisor 8
        // Actual timeslice of motion 32.768 ms
        // Pulse width base 500 ns
        #define TIMESLICE_32MS  2

    #define MOVEMENT_TIMESLICE TIMESLICE_32MS

#define MAX_JOG_RATE 0x30
#define IDLE_TIMESLICE_COUNT_BEFORE_DISABLE 0xff

// This is multiplied with the pulse width base value to get the
// stepping pulse width.
    #define STEP_PULSE_WIDTH_TICKS      10

// Number of timeslices before sending the status
// Zero means 256
// Comment out to disable entirely
    //#define STATUS_PUSH_INTERVAL        10

// Select USART to use
    // #define USE_USART0
    #define USE_USART1
    // #define USE_USART2
    // #define USE_USART3

// Set ESTOP flag after reset. Comment to disable
    #define ESTOP_AFTER_RESET

// Please keep these a power of two or hilarity ensues (value <= 256)
//    #define SERIAL_TX_QUEUE_SIZE        64
//    #define SERIAL_RX_QUEUE_SIZE        64


// Please keep this a power of two or hilarity ensues (value <= 256)
#define MOVEMENT_QUEUE_SIZE         256

// -----------------------------------------------------------------
//
//  Configuration for inputs
//
//      BOTH asserting or deasserting a limit or probe signal stop the
//      machine. This behaviour makes homing easier.
//
//      Pull-ups are enabled, so whether you use normally closed switches
//      or not, you should wire them to the ground. I'd recommend that
//      you use them so a wire failure will trip the limit.
//
//      Default values result in the following:
//          LIMIT_X  = PK.0
//          LIMIT_Y  = PK.1
//          LIMIT_Z  = PK.2
//          PROBE    = PK.3
//          /ESTOP   = PK.4
//
// -----------------------------------------------------------------

// Input polarity for ESTOP. CMD_CLEAR_ESTOP is rejected if ESTOP is active.
    #define INPUT_ESTOP_ACTIVE_LOW
    #define INPUT_LIMITS_ACTIVE_LOW
    #define INPUT_PROBE_ACTIVE_LOW

// Port definition
    #define INPUT_PORT          PORTK
    #define INPUT_DDR           DDRK
    #define INPUT_PIN           PINK
    #define INPUT_PCIE          PCIE2
    #define INPUT_PCIMSK        PCMSK2
    #define INPUT_PCI_vect      PCINT2_vect

// Pin definitions
    #define INPUT_LIMIT_X       0
    #define INPUT_LIMIT_Y       1
    #define INPUT_LIMIT_Z       2
    #define INPUT_PROBE         3
    #define INPUT_ESTOP         4

// -----------------------------------------------------------------
//
//  Configuration for motor outputs
//
//      Default values result in the following:
//          /STEP_X  = PF.0
//          /DIR_X   = PF.1
//          /STEP_Y  = PF.2
//          /DIR_Y   = PF.3
//          /STEP_Z  = PF.4
//          /DIR_Z   = PF.5
//          /STEP_A  = PF.6
//          /DIR_A   = PF.7


//          /ENABLE  = PA.0
//          /SPINDLE = PA.1
//
// -----------------------------------------------------------------

#define STEPPERS_DISABLED_IN_ESTOP

// Signal polarity, comment to disable
    #define CONTROL_ENABLE_ACTIVE_LOW
//    #define CONTROL_SPINDLE_ACTIVE_LOW

    #define STEPPER_STEP_ACTIVE_LOW
    #define STEPPER_DIR_POSITIVE_LOW

// Port definition
    #define STEPPER_PORT        PORTF
    #define STEPPER_DDR         DDRF
    #define CONTROL_PORT        PORTA
    #define CONTROL_DDR         DDRA


// Pin definitions
    #define STEPPER_PIN_STEP_X  0
    #define STEPPER_PIN_DIR_X   1

    #define STEPPER_PIN_STEP_Y  2
    #define STEPPER_PIN_DIR_Y   3
    #define STEPPER_PIN_STEP_Z  4
    #define STEPPER_PIN_DIR_Z   5
    #define STEPPER_PIN_STEP_A  6
    #define STEPPER_PIN_DIR_A   7

    #define CONTROL_PIN_ENABLE  0
    #define CONTROL_PIN_SPINDLE 1

// -----------------------------------------------------------------
//
//  Definitions for USARTs follow
//  There shouldn't be any need to change these normally
//
// -----------------------------------------------------------------

#ifdef USE_USART0
    #define SERIAL_UDRE_vect        USART0_UDRE_vect
    #define SERIAL_RX_vect          USART0_RX_vect
    #define SERIAL_UDR              UDR0
    #define SERIAL_UBRR             UBRR0
    #define SERIAL_UCSRnA           UCSR0A
    #define SERIAL_UCSRnB           UCSR0B
    // These should be equal for all serial interfaces, but let's define them anyway
    #define SERIAL_U2X              U2X0
    #define SERIAL_UDREn            UDRE0
    #define SERIAL_RXCn             RXC0
    #define SERIAL_RXEN             RXEN0
    #define SERIAL_TXEN             TXEN0
    #define SERIAL_RXCIE            RXCIE0
    #define SERIAL_UDRIE            UDRIE0
#endif
#ifdef USE_USART1
    #define SERIAL_UDRE_vect        USART1_UDRE_vect
    #define SERIAL_RX_vect          USART1_RX_vect
    #define SERIAL_UDR              UDR1
    #define SERIAL_UBRR             UBRR1
    #define SERIAL_UCSRnA           UCSR1A
    #define SERIAL_UCSRnB           UCSR1B
    #define SERIAL_U2X              U2X1
    #define SERIAL_UDREn            UDRE1
    #define SERIAL_RXCn             RXC1
    #define SERIAL_RXEN             RXEN1
    #define SERIAL_TXEN             TXEN1
    #define SERIAL_RXCIE            RXCIE1
    #define SERIAL_UDRIE            UDRIE1
#endif
#ifdef USE_USART2
    #define SERIAL_UDRE_vect        USART2_UDRE_vect
    #define SERIAL_RX_vect          USART2_RX_vect
    #define SERIAL_UDR              UDR2
    #define SERIAL_UBRR             UBRR2
    #define SERIAL_UCSRnA           UCSR2A
    #define SERIAL_UCSRnB           UCSR2B
    #define SERIAL_U2X              U2X2
    #define SERIAL_UDREn            UDRE2
    #define SERIAL_RXCn             RXC2
    #define SERIAL_RXEN             RXEN2
    #define SERIAL_TXEN             TXEN2
    #define SERIAL_RXCIE            RXCIE2
    #define SERIAL_UDRIE            UDRIE2
#endif
#ifdef USE_USART3
    #define SERIAL_UDRE_vect        USART3_UDRE_vect
    #define SERIAL_RX_vect          USART3_RX_vect
    #define SERIAL_UDR              UDR3
    #define SERIAL_UBRR             UBRR3
    #define SERIAL_UCSRnA           UCSR3A
    #define SERIAL_UCSRnB           UCSR3B
    #define SERIAL_U2X              U2X3
    #define SERIAL_UDREn            UDRE3
    #define SERIAL_RXCn             RXC3
    #define SERIAL_RXEN             RXEN3
    #define SERIAL_TXEN             TXEN3
    #define SERIAL_RXCIE            RXCIE3
    #define SERIAL_UDRIE            UDRIE3
#endif

// -----------------------------------------------------------------
//
// Internal defines
// Defined here only because this is a common include file
//
// -----------------------------------------------------------------

#define STATE_FLAGS  GPIOR0
#define SYS_FLAGS_A  GPIOR1

#define EVENT_TIMESLICE 0

#define FIRMWARE_PANIC() for (;;) { cli(); UDR1 = 0xEE; UDR0 = 0xEE; }
#endif

