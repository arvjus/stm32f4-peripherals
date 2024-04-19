
#include <stdint.h>
#include <stdio.h>

#include "stm32f407xx.h"

#define LEDDELAY    1000
int main(void);

int main(void)
{
    /* Enable GPIOD clock (AHB1ENR: bit 3) */
    // AHB1ENR: XXXX XXXX XXXX XXXX XXXX XXXX XXXX 1XXX
    RCC->AHB1ENR |= 0x00000008;

    GPIOD->MODER &= 0xFCFFFFFF;   // Reset bits 25:24 to clear old values
    GPIOD->MODER |= 0x01000000;   // Set MODER bits 25:24 to 01

    /* Set or clear pins (ODR: bit 12) */
    // Set pin 12 to 1 to turn on an LED
    // ODR: xxx1 XXXX XXXX XXXX
    GPIOD->ODR |= 0x1000;

    while(1)
    {
        delay(LEDDELAY);
        GPIOD->ODR ^= (1U << 12);  // Toggle LED
    }

    __asm("NOP"); // Assembly inline can be used if needed
    return 0;
}
