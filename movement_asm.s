#include <avr/io.h>

#define FROM_ASM_CODE
#include "config.h"

.macro BEGIN_FUNC FUNC
.section .text.\FUNC, "ax", @progbits
.global \FUNC	
.type	\FUNC, @function
\FUNC:
.endm


BEGIN_FUNC TIMER3_COMPA_vect
    // Deassert stepper bit
#ifdef STEPPER_STEP_ACTIVE_LOW
    sbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_X
#else
    cbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_X
#endif
    reti

BEGIN_FUNC TIMER3_COMPB_vect
#ifdef STEPPER_STEP_ACTIVE_LOW
    sbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_Y
    sbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_A
#else
    cbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_Y
    cbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_A
#endif
    reti

BEGIN_FUNC TIMER3_COMPC_vect
#ifdef STEPPER_STEP_ACTIVE_LOW
    sbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_Z
#else
    cbi _SFR_IO_ADDR(STEPPER_PORT), STEPPER_PIN_STEP_Z
#endif
    reti
