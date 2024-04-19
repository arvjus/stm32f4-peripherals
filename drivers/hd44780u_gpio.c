
#include <stdlib.h>
#include <string.h>
#include "hd44780u_gpio.h"

#define RS_CMD  0x0
#define RS_DATA 0x1

#define BIT_DISPLAY 2
#define BIT_CURSOR  1
#define BIT_BLINK   0


static void send_4bits(LCD_Handle_t *pLCDHandle, uint8_t data) {
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinD4, (data >> 0) & 0x1);
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinD5, (data >> 1) & 0x1);
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinD6, (data >> 2) & 0x1);
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinD7, (data >> 3) & 0x1);

    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinEN, 0x0);
    udelay(1);
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinEN, 0x1);
    udelay(1);
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinEN, 0x0);
    udelay(50);
}

static void send_cmd(LCD_Handle_t *pLCDHandle, uint8_t cmd) {
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinRS, RS_CMD);
    send_4bits(pLCDHandle, cmd >> 4);
    send_4bits(pLCDHandle, cmd & 0x0f);
}

static void display_ctrl(LCD_Handle_t *pLCDHandle, uint8_t bit, uint8_t on_off)
{
    if (on_off)
        pLCDHandle->displayCtrl |= (1 << bit);
    else
        pLCDHandle->displayCtrl &= ~(1 << bit);
    send_cmd(pLCDHandle, pLCDHandle->displayCtrl);
}

void LCD_Init(LCD_Handle_t *pLCDHandle)
{
    // init GPIO
    GPIO_Handle_t gpio;
    memset(&gpio, 0, sizeof gpio);

    gpio.pGPIOx = pLCDHandle->pGPIOx;
    gpio.pinConfig.mode = GPIO_PIN_MODE_OUT;
    gpio.pinConfig.opType = GPIO_PIN_OPTYPE_PP;
    gpio.pinConfig.number = pLCDHandle->config.pinRS;
    GPIO_Init(&gpio);
    gpio.pinConfig.number = pLCDHandle->config.pinEN;
    GPIO_Init(&gpio);
    gpio.pinConfig.number = pLCDHandle->config.pinD4;
    GPIO_Init(&gpio);
    gpio.pinConfig.number = pLCDHandle->config.pinD5;
    GPIO_Init(&gpio);
    gpio.pinConfig.number = pLCDHandle->config.pinD6;
    GPIO_Init(&gpio);
    gpio.pinConfig.number = pLCDHandle->config.pinD7;
    GPIO_Init(&gpio);

    // init HD44780U
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinRS, RS_CMD);

    delay(40);
    send_4bits(pLCDHandle, 0b0011);

    delay(5);
    send_4bits(pLCDHandle, 0b0011);

    udelay(100);
    send_4bits(pLCDHandle, 0b0011);
    send_4bits(pLCDHandle, 0b0010);

    // function set
    send_cmd(pLCDHandle, pLCDHandle->config.lines == 2 ? 0b00101000 : 0b00100000);

    // display on/off control
    pLCDHandle->displayCtrl = 0b00001100;
    send_cmd(pLCDHandle, pLCDHandle->displayCtrl);

    // clear display
    LCD_Clear(pLCDHandle);

    // entry mode set
    send_cmd(pLCDHandle, 0b00000110);
}

void LCD_Clear(LCD_Handle_t *pLCDHandle)
{
    send_cmd(pLCDHandle, 0b00000001);
    delay(2);
}

void LCD_Home(LCD_Handle_t *pLCDHandle)
{
    send_cmd(pLCDHandle, 0b00000010);
    delay(2);
}

void LCD_DisplCtrl(LCD_Handle_t *pLCDHandle, uint8_t on_off)
{
    display_ctrl(pLCDHandle, BIT_DISPLAY, on_off);
}

void LCD_CursorCtrl(LCD_Handle_t *pLCDHandle, uint8_t on_off)
{
    display_ctrl(pLCDHandle, BIT_CURSOR, on_off);
}

void LCD_BlinkCtrl(LCD_Handle_t *pLCDHandle, uint8_t on_off)
{
    display_ctrl(pLCDHandle, BIT_BLINK, on_off);
}

void LCD_SetCursor(LCD_Handle_t *pLCDHandle, uint8_t col, uint8_t row)
{
    send_cmd(pLCDHandle, (row == 1 ? 0b10000000 : 0b11000000) | --col);
}

void LCD_PrintChar(LCD_Handle_t *pLCDHandle, char ch)
{
    GPIO_WriteToOutputPin(pLCDHandle->pGPIOx, pLCDHandle->config.pinRS, RS_DATA);
    send_4bits(pLCDHandle, ch >> 4);
    send_4bits(pLCDHandle, ch & 0x0f);
}

void LCD_PrintString(LCD_Handle_t *pLCDHandle, char *str)
{
    for (; *str; str++)
        LCD_PrintChar(pLCDHandle, *str);
}

void LCD_PrintNumber(LCD_Handle_t *pLCDHandle, uint32_t num)
{
    static char buff[10];
    itoa(num, buff, 10);
    LCD_PrintString(pLCDHandle, buff);
}
