
#include <string.h>
#include "hd44780u_gpio.h"

int main(void)
{
    int32_t count = 1;

    LCD_Handle_t lcd;
    memset(&lcd, 0, sizeof lcd);

    lcd.config.lines = 2;
    lcd.config.pinEN = 0;
    lcd.config.pinRS = 1;
    lcd.config.pinD4 = 2;
    lcd.config.pinD5 = 3;
    lcd.config.pinD6 = 4;
    lcd.config.pinD7 = 5;
    lcd.pGPIOx = GPIOE;
    LCD_Init(&lcd);

    LCD_Home(&lcd);
    LCD_PrintString(&lcd, "I'm a counter!");
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
