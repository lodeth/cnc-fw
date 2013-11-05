CC         = avr-gcc
AVRDUDE    = avrdude
AVR_OBJCPY = avr-objcopy
AVR_SIZE   = avr-size

DEVICE     = atmega2560
FREQ       = 16000000
PROGDEV    = avrisp2
PROGTTY    = /dev/tty.usbmodemfd121

CFLAGS     = -Wall -Os -I.
OBJECTS    = main.o movement.o movement_asm.o precalc.o serial.o

PROGCMD    = $(AVRDUDE) -B10 -p $(DEVICE) -c $(PROGDEV) -P $(PROGTTY)
COMPILE    = $(CC) $(CFLAGS) -DF_CPU=$(FREQ) -mmcu=$(DEVICE) 

all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@
	@$(COMPILE) -MM  $< > $*.d

.s.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

install: all
	$(PROGCMD) -U flash:w:main.hex:i

clean:
	rm -f main.hex main.elf $(OBJECTS) $(OBJECTS:.o=.d)

main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS) -lm -Wl,--gc-sections

main.hex: main.elf
	rm -f main.hex
	$(AVR_OBJCPY) -j .text -j .data -O ihex main.elf main.hex
	$(AVR_SIZE) --format=berkeley main.elf

.PHONY: all install

# include generated header dependencies
-include $(OBJECTS:.o=.d)

