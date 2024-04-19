
#include <string.h>
#include "hd44780u_i2c.h"

#define    PIN_SCL        6   // PB6
#define    PIN_SDA        7   // PB7

int main(void)
{
    RCC->CFGR &= ~(0x3);    // HSI

    int32_t count = 1;

    LCD_Handle_t lcd;
    memset(&lcd, 0, sizeof lcd);

    lcd.config.lines = 2;
    lcd.config.backLight = true;
    lcd.config.pinScl = PIN_SCL;
    lcd.config.pinSda = PIN_SDA;
    lcd.pGPIOx = GPIOB;
    lcd.i2c.pI2Cx = I2C1;
    LCD_Init(&lcd);

    LCD_Home(&lcd);
    LCD_PrintString(&lcd, "I'm a counter!");
    LCD_SetCursor(&lcd, 1, 3);
    LCD_PrintString(&lcd, "3rd line");
    LCD_SetCursor(&lcd, 1, 4);
    LCD_PrintString(&lcd, "4th line");
    while (1) {
        delay(1000);

        if (count == 10) {
            LCD_CursorCtrl(&lcd, 1);
            LCD_SetCursor(&lcd, 1, 1);
            LCD_PrintString(&lcd, "cursor on     ");
        }
        if (count == 15) {
            LCD_BlinkCtrl(&lcd, 1);
            LCD_SetCursor(&lcd, 1, 1);
            LCD_PrintString(&lcd, "blink on      ");
        }
        if (count == 20) {
            LCD_BlinkCtrl(&lcd, 0);
            LCD_SetCursor(&lcd, 1, 1);
            LCD_PrintString(&lcd, "blink off     ");
        }
        if (count == 25) {
            LCD_CursorCtrl(&lcd, 0);
            LCD_SetCursor(&lcd, 1, 1);
            LCD_PrintString(&lcd, "cursor off    ");
        }
        if (count == 30)
            LCD_DisplCtrl(&lcd, 0);
        if (count == 35) {
            LCD_DisplCtrl(&lcd, 1);
            LCD_SetCursor(&lcd, 1, 1);
            LCD_PrintString(&lcd, "display on    ");
        }

        LCD_SetCursor(&lcd, 1, 2);
        LCD_PrintNumber(&lcd, count ++);
        LCD_PrintString(&lcd, " sec");
    }

    return 0;
}
