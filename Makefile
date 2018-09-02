CC=avr-gcc
OBJDIR=obj

SRC = $(wildcard *.c)
OBJ = $(addprefix $(OBJDIR)/, $(notdir $(SRC:.c=.o)))

CFLAGS = -std=c99 -Wall -Werror -Wno-unused -Os -g -D_GNU_SOURCE -mmcu=atmega328p -DF_CPU=16000000UL

shifter.elf: $(OBJ)
	$(CC) $(OBJ) $(CFLAGS) -o $@
	rm -f shifter.hex
	avr-objcopy -j .text -j .data -O ihex shifter.elf shifter.hex

obj/%.o: %.c | obj
	$(CC) $< -c $(CFLAGS) -o $@


obj:
	mkdir $(OBJDIR)

clean:
	rm $(OBJ)
