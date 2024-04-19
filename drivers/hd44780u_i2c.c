
#include <string.h>
#include <stdlib.h>
#include "hd44780u_i2c.h"

// Pins connected to HD44780u
// D7 D6 D5 D4 Bl En Rw Rs

#define SLAVE_ADDR      0x27

#define RS_CMD  0x0
#define RS_DATA 0x1

#define BIT_BL      3
#define BIT_EN      2

#define BIT_DISPLAY 2
#define BIT_CURSOR  1
#define BIT_BLINK   0

static void lcd_write_4bits(I2C_Handle_t *pI2CHandle, uint8_t data) {
    I2C_MasterSendData(pI2CHandle,&data,1,SLAVE_ADDR,I2C_SR_EN);
    data |= (1 << BIT_EN);       // En
    I2C_MasterSendData(pI2CHandle,&data,1,SLAVE_ADDR,I2C_SR_EN);
    data &= ~(1 << BIT_EN);      // En
    I2C_MasterSendData(pI2CHandle,&data,1,SLAVE_ADDR,I2C_SR_DI);
    udelay(50);
}

static void lcd_write(I2C_Handle_t *pI2CHandle, uint8_t data, uint8_t mode) {
    lcd_write_4bits(pI2CHandle, mode | (data & 0xf0));
    lcd_write_4bits(pI2CHandle, mode | ((data << 4) & 0xf0));
}

void LCD_Init(LCD_Handle_t *pLCDHandle)
{
    // GPIO
    GPIO_Handle_t i2c1Pins;
    memset(&i2c1Pins, 0, sizeof(i2c1Pins));

    i2c1Pins.pGPIOx = pLCDHandle->pGPIOx;
    i2c1Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
    i2c1Pins.pinConfig.altFunMode = 4;
    i2c1Pins.pinConfig.opType = GPIO_PIN_OPTYPE_OD;
    i2c1Pins.pinConfig.puPdControl = GPIO_PIN_PU;
    i2c1Pins.pinConfig.speed = GPIO_PIN_SPEED_MEDIUM;
    // SCL
    i2c1Pins.pinConfig.number = pLCDHandle->config.pinScl;
    GPIO_Init(&i2c1Pins);
    // SDA
    i2c1Pins.pinConfig.number = pLCDHandle->config.pinSda;
    GPIO_Init(&i2c1Pins);

    // I2C
    pLCDHandle->i2c.config.ackControl = I2C_ACK_EN;
    pLCDHandle->i2c.config.fmDutyCycle = I2C_FM_DUTY_2;
    pLCDHandle->i2c.config.sclSpeed = I2C_SCL_SPEED_SM;
    I2C_Init(&pLCDHandle->i2c);

    I2C_PeriphCtrl(pLCDHandle->i2c.pI2Cx, ENABLE);

    // init HD44780U
    delay(100);
    lcd_write_4bits(&pLCDHandle->i2c, 0b0011 << 4);

    delay(5);
    lcd_write_4bits(&pLCDHandle->i2c, 0b0011 << 4);

    lcd_write_4bits(&pLCDHandle->i2c, 0b0011 << 4);
    lcd_write_4bits(&pLCDHandle->i2c, 0b0010 << 4);

    // function set
    lcd_write(&pLCDHandle->i2c, pLCDHandle->config.lines == 1 ? 0b00100000 : 0b00101000, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));

    // display on/off control
    pLCDHandle->displayCtrl = 0b00001100;
    lcd_write(&pLCDHandle->i2c, pLCDHandle->displayCtrl, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));

    // clear display
    LCD_Clear(pLCDHandle);

    // entry mode set
    lcd_write(&pLCDHandle->i2c, 0b00000110, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));
 }

void LCD_Clear(LCD_Handle_t *pLCDHandle)
{
    lcd_write(&pLCDHandle->i2c, 0b00000001, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));
    delay(5);
}

void LCD_Home(LCD_Handle_t *pLCDHandle)
{
    lcd_write(&pLCDHandle->i2c, 0b00000010, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));
    delay(5);
}

static void display_ctrl(LCD_Handle_t *pLCDHandle, uint8_t bit, uint8_t on_off)
{
    if (on_off)
        pLCDHandle->displayCtrl |= (1 << bit);
    else
        pLCDHandle->displayCtrl &= ~(1 << bit);
    lcd_write(&pLCDHandle->i2c, pLCDHandle->displayCtrl, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));
}

void LCD_DisplCtrl(LCD_Handle_t *pLCDHandle, bool on_off)
{
    display_ctrl(pLCDHandle, BIT_DISPLAY, on_off);
}

void LCD_CursorCtrl(LCD_Handle_t *pLCDHandle, bool on_off)
{
    display_ctrl(pLCDHandle, BIT_CURSOR, on_off);
}

void LCD_BlinkCtrl(LCD_Handle_t *pLCDHandle, bool on_off)
{
    display_ctrl(pLCDHandle, BIT_BLINK, on_off);
}

void LCD_SetCursor(LCD_Handle_t *pLCDHandle, uint8_t col, uint8_t row)
{
    uint8_t row_addr = 0x80;
    switch (row) {
        case 2: row_addr = 0xc0; break;
        case 3: row_addr = 0x94; break;
        case 4: row_addr = 0xd4; break;
    }
    lcd_write(&pLCDHandle->i2c, row_addr | --col, RS_CMD | (pLCDHandle->config.backLight << BIT_BL));
}

void LCD_PrintChar(LCD_Handle_t *pLCDHandle, char ch)
{
    lcd_write(&pLCDHandle->i2c, ch, RS_DATA | (pLCDHandle->config.backLight << BIT_BL));
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
