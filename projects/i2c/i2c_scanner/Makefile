
SOURCES=i2c_scanner.c \
../../../common/startup.c \
../../../common/syscalls_usart.c \
../../../drivers/stm32f407xx.c \
../../../drivers/stm32f407xx_gpio.c \
../../../drivers/stm32f407xx_i2c.c \
../../../drivers/stm32f407xx_usart.c

OBJECTS=$(SOURCES:.c=.o)

$(info OBJECTS is $(OBJECTS))

CC = arm-none-eabi-gcc
CPU = cortex-m4
CFLAGS= -mcpu=$(CPU) -mthumb -std=gnu11 -O0 -Wall -I. -I../../../drivers -I../../../common -g -c
LDFLAGS= -mcpu=$(CPU) -mthumb -mfloat-abi=soft --specs=nano.specs -T../../../board/linker.ld -Wl,-Map=i2c_scanner.map

all: i2c_scanner.elf

i2c_scanner.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) i2c_scanner.elf i2c_scanner.map
