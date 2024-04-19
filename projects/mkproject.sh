#!/bin/sh

test -n "$1" || { echo "usage: $0 NAME"; exit 1; }

NAME=$1
mkdir -p $NAME

cat >$NAME/${NAME}.c<<EOF1

#include <stdint.h>
#include <stdio.h>
#include "stm32f407xx.h"

extern void stdio_usart_init(GPIO_RegDef_t *pGPIOx, uint8_t pinTx, uint8_t pinRx, USART_RegDef_t *pUSARTx);

int main(void)
{
    RCC_SysClkSwitch(RCC_SYSCLK_HSE);
    stdio_usart_init(GPIOA, 2, 3, USART2);


    return 0;
}
EOF1

cat >$NAME/Makefile<<EOF2

SOURCES=${NAME}.c \\
../../../common/startup.c \\
../../../common/syscalls.c \\
../../../drivers/stm32f407xx.c \\
../../../drivers/stm32f407xx_gpio.c \\
../../../drivers/stm32f407xx_i2c.c \\
../../../drivers/stm32f407xx_spi.c \\
../../../drivers/stm32f407xx_usart.c

OBJECTS=\$(SOURCES:.c=.o)

\$(info OBJECTS is \$(OBJECTS))

CC = arm-none-eabi-gcc
CPU = cortex-m4
CFLAGS= -mcpu=\$(CPU) -mthumb -std=gnu11 -O0 -Wall -I. -I../../../drivers -I../../../common -g -c
LDFLAGS= -mcpu=\$(CPU) -mthumb -mfloat-abi=soft --specs=nano.specs -T../../../board/linker.ld -Wl,-Map=${NAME}.map

all: ${NAME}.elf

${NAME}.elf: \$(OBJECTS)
	\$(CC) \$(LDFLAGS) \$(OBJECTS) -o \$@

%.o: %.c
	\$(CC) \$(CFLAGS) $< -o \$@

clean:
	rm -f \$(OBJECTS) ${NAME}.elf ${NAME}.map
EOF2
