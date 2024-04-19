#ifndef STM32F4_PERIPHERALS_SDCARD_SPI_H
#define STM32F4_PERIPHERALS_SDCARD_SPI_H

#include <stdbool.h>
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"

typedef struct {
    uint8_t pinNss;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinSclk;		/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinMiso;		/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinMosi;		/*!< possible values 0-15 for given GPIOx >*/
} SDCARD_Config_t;

typedef struct {
    SDCARD_Config_t config;		// configuration settings
    GPIO_RegDef_t  *pGPIOx;		// base address to GPIO peripheral
    SPI_Handle_t    spi;
} SDCARD_Handle_t;

void SDCARD_Init(SDCARD_Handle_t *pCDCARDHandle);
void SDCARD_Ctrl(SDCARD_Handle_t *pCDCARDHandle, uint8_t en_di);

#endif //STM32F4_PERIPHERALS_SDCARD_SPI_H
