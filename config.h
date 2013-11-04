#ifndef CONFIG_H
#define CONFIG_H

#define STEPPER_PORT PORTF
#define STEPPER_DDR  DDRF
#define STEP_PULSE_WIDTH_TICKS 10

#define INPUT_PORT          PORTK
#define INPUT_DDR           DDRK
#define INPUT_PIN           PINK

#define INPUT_PCIE          PCIE2
#define INPUT_PCIMSK        PCMSK2
#define INPUT_PCI_vect      PCINT2_vect
#define INPUT_LIMIT_MASK    0x07    

#define AXIS_BIT_X 0
#define AXIS_BIT_Y 1
#define AXIS_BIT_Z 2


#endif

