#ifndef STM32F4_PERIPHERALS_HD44780U_LCD_H
#define STM32F4_PERIPHERALS_HD44780U_LCD_H

#include "stm32f407xx_gpio.h"

typedef struct {
    uint8_t lines;			/*!< possible values 1,2 >*/
    uint8_t pinRS;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinEN;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinD4;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinD5;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinD6;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinD7;			/*!< possible values 0-15 for given GPIOx >*/
} LCD_Config_t;

typedef struct {
    LCD_Config_t config;		// configuration settings
    GPIO_RegDef_t *pGPIOx;		// base address to GPIO peripheral
    uint8_t displayCtrl;		// display/cursor/blink bits, default 1/0/0
} LCD_Handle_t;

void LCD_Init(LCD_Handle_t *pLCDHandle);
void LCD_Clear(LCD_Handle_t *pLCDHandle);
void LCD_Home(LCD_Handle_t *pLCDHandle);
void LCD_DisplCtrl(LCD_Handle_t *pLCDHandle, uint8_t on_off);
void LCD_CursorCtrl(LCD_Handle_t *pLCDHandle, uint8_t on_off);
void LCD_BlinkCtrl(LCD_Handle_t *pLCDHandle, uint8_t on_off);
void LCD_SetCursor(LCD_Handle_t *pLCDHandle, uint8_t col, uint8_t row);
void LCD_PrintChar(LCD_Handle_t *pLCDHandle, char ch);
void LCD_PrintString(LCD_Handle_t *pLCDHandle, char *str);
void LCD_PrintNumber(LCD_Handle_t *pLCDHandle, uint32_t num);

#endif //STM32F4_PERIPHERALS_HD44780U_LCD_H
