#ifndef I2C_SCANNER_HD44780U_I2C_H
#define I2C_SCANNER_HD44780U_I2C_H

#include "stm32f407xx_gpio.h"
#include "stm32f407xx_i2c.h"

typedef struct {
    uint8_t lines;			/*!< possible values 1,2 >*/
    bool    backLight;      /*!< possible values false, true >*/
    uint8_t pinScl;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinSda;			/*!< possible values 0-15 for given GPIOx >*/
} LCD_Config_t;

typedef struct {
    LCD_Config_t    config;		// configuration settings
    GPIO_RegDef_t  *pGPIOx;		// base address to GPIO peripheral
    I2C_Handle_t    i2c;
    uint8_t displayCtrl;		// display/cursor/blink bits, default 1/0/0
} LCD_Handle_t;

void LCD_Init(LCD_Handle_t *pLCDHandle);
void LCD_Clear(LCD_Handle_t *pLCDHandle);
void LCD_Home(LCD_Handle_t *pLCDHandle);
void LCD_DisplCtrl(LCD_Handle_t *pLCDHandle, bool on_off);
void LCD_CursorCtrl(LCD_Handle_t *pLCDHandle, bool on_off);
void LCD_BlinkCtrl(LCD_Handle_t *pLCDHandle, bool on_off);
void LCD_SetCursor(LCD_Handle_t *pLCDHandle, uint8_t col, uint8_t row);
void LCD_PrintChar(LCD_Handle_t *pLCDHandle, char ch);
void LCD_PrintString(LCD_Handle_t *pLCDHandle, char *str);
void LCD_PrintNumber(LCD_Handle_t *pLCDHandle, uint32_t num);

#endif //I2C_SCANNER_HD44780U_I2C_H
