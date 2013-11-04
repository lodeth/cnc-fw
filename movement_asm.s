#include <avr/io.h>

#define FROM_ASM_CODE
#include "config.h"

.macro ENDPULSE_ISR VECTOR STEP_BIT CHANNEL_BIT
// Timer3 channel A for X axis
.section .text.\VECTOR, "ax", @progbits
.global \VECTOR	
.type	\VECTOR, @function
\VECTOR:
    // Output high on stepper bit X
    sbi _SFR_IO_ADDR(STEPPER_PORT), \STEP_BIT

    // Stop timer3 channel A from repeating
    // TIMSK3 is too high to access via IO space and use cbi, otherwise this would be a lot shorter :(
    // Namely the entire handler would be be:
    // TIMER3_COMPA_vect:
    //     sbi _SFR_IO_ADDR(STEPPER_PORT), AXIS_BIT_X
    //     cbi _SFR_IO_ADDR(TIMSK3), OCIE3A
    //     iret

    // Save flags to r0
	push r0
	lds r0, SREG

    // Clear the bit
    push r16
    lds r16, TIMSK3
    cbr r16, 1<<\CHANNEL_BIT 
    sts TIMSK3, r16 
    pop r16

    // Restore flags
	sts SREG, r0
	pop r0

	reti
	.size	\VECTOR, .-\VECTOR
.endm

ENDPULSE_ISR TIMER3_COMPA_vect, AXIS_BIT_X, OCIE3A
ENDPULSE_ISR TIMER3_COMPB_vect, AXIS_BIT_Y, OCIE3B
ENDPULSE_ISR TIMER3_COMPC_vect, AXIS_BIT_Z, OCIE3C
