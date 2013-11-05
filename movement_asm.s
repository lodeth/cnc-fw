#include <avr/io.h>

#define FROM_ASM_CODE
#include "config.h"

.macro ENDPULSE_ISR VECTOR STEP_BIT CHANNEL_BIT
.section .text.\VECTOR, "ax", @progbits
.global \VECTOR	
.type	\VECTOR, @function
\VECTOR:
    // Deassert stepper bit
#ifdef STEPPER_STEP_ACTIVE_LOW
    sbi _SFR_IO_ADDR(STEPPER_PORT), \STEP_BIT
#else
    cbi _SFR_IO_ADDR(STEPPER_PORT), \STEP_BIT
#endif
    reti
	.size	\VECTOR, .-\VECTOR
.endm

ENDPULSE_ISR TIMER3_COMPA_vect, STEPPER_PIN_STEP_X, OCIE3A
ENDPULSE_ISR TIMER3_COMPB_vect, STEPPER_PIN_STEP_Y, OCIE3B
ENDPULSE_ISR TIMER3_COMPC_vect, STEPPER_PIN_STEP_Z, OCIE3C

