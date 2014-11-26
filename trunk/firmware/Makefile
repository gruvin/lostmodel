PRG           = lostmodel
OBJ           = lostmodel.o
MCU_TARGET    = atmega88p
OPTIMIZE      = -Os

DEFS          = -DF_CPU=8000000 #-DDEBUG_LEVEL=2
LIBS          =

# You should not have to change anything below here.

CC            = avr-gcc
CPP           = avr-g++

# Override is only needed by avr-lib build system.

override CPPFLAGS      = -ggdb -Wall $(OPTIMIZE) -I. -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -I. -Wl,-Map,$(PRG).map

OBJCOPY       = avr-objcopy
OBJDUMP       = avr-objdump
AVRDUDE       = avrdude -c avrispmkii -P usb -B2  -p $(MCU_TARGET)

all: $(PRG).elf hex lst 

$(PRG).elf: $(OBJ)
	$(CPP) $(CPPFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# dependency:
%.o: %.cpp %.h
	$(CPP) $(CPPFLAGS) $(LDFLAGS) -c -o $@ $<

clean:
	rm -f *.o *.elf *.eps *.png *.pdf *.bak *.lst *.map *.hex *.bin

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rule for building rom image

hex: $(PRG).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	./checksize $< 

flash:	all
	$(AVRDUDE) -U flash:w:$(PRG).hex:i

fuses:	# internal 8MHz clock, brown-out detection 4.3V, 65ms boot delay, EESAVE enabled
	$(AVRDUDE) -B8 -u -U lfuse:w:0xe2:m -U hfuse:w:0xd5:m -U efuse:w:0xf9:m

