# Name: Makefile
# Project: PowerSwitch
# Author: Christian Starkjohann
# Creation Date: 2004-12-29
# Tabsize: 4
# Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
# This Revision: $Id$

DEVICE = atmega88p
AVRDUDE = avrdude -c avrispmkii -P usb -B2  -p $(DEVICE)
# Choose your favorite programmer and interface above.

COMPILE = avr-gcc -g -Wall -Os -I. -mmcu=$(DEVICE) -DF_CPU=8000000 #-DDEBUG_LEVEL=2
# NEVER compile the final product with debugging! Any debug output will
# distort timing so that the specs can't be met.

OBJECTS = main.o

# symbolic targets:
all:	main.hex

%.o: %.cpp %.h
	$(COMPILE) -c $< -o $@

%.o: %.S
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

%.o:  %.s
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i


fuses:	# ATmega88P - internal 8MHz clock, brown-out detection 4.3V, 65ms boot delay, EESAVE enabled
	$(AVRDUDE) -B8 -u -U lfuse:w:0xe2:m -U hfuse:w:0xd5:m -U efuse:w:0xf9:m


clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep.hex main.elf *.o

# file targets:
main.elf:	$(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.lst:	main.elf
	avr-objdump -dS main.elf > main.lst

main.hex:	main.elf main.lst
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	./checksize main.elf
# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

disasm:	main.elf
	avr-objdump -dS main.elf
