TARGET=iron-pulse.elf
MAP=iron-pulse.map
MCU=attiny85
SRCS=$(wildcard *.c)
CC=avr-gcc
AVRDUDE=avrdude
CFLAGS=-Os -Wall -Wextra -mmcu=$(MCU) -std=c99 -fshort-enums
LFLAGS=-mmcu=$(MCU) -Wl,-M=$(MAP)
ADUDFLAGS=-p $(MCU) -c avrftdi
LIBS=


OBJDIR=obj
OBJS=$(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

# Main target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) $(LIBS) -o $(TARGET)

.PHONY: flash
flash: $(TARGET)
	$(AVRDUDE) $(ADUDFLAGS) -U flash:w:$(TARGET)

.PHONY: fuse
fuse: $(TARGET)
	$(AVRDUDE) $(ADUDFLAGS) -U hfuse:w:$(TARGET)
	$(AVRDUDE) $(ADUDFLAGS) -U lfuse:w:$(TARGET)

# Build object and dependency files from C sources
$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) -c -MMD -MP $(CFLAGS) $< -o $@
		
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Delete intermediate files
.PHONY: clean
clean:
	rm -rf $(OBJDIR)

# Delete intermediate and output files
.PHONY: mrproper
mrproper: | clean
	rm -f $(MAP)
	rm -f $(TARGET)

# Include auto-generated dependencies
-include $(SRCS:%.c=$(OBJDIR)/%.d)

