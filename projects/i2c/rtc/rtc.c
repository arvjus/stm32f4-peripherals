
#include <string.h>
#include "stm32f407xx_gpio.h"
#include "ds1307_rtc.h"

#define    PIN_SCL        6   // PB6
#define    PIN_SDA        7   // PB7

RTC_Handle_t rtcHandle;

void setupGPIOForButton() {
    GPIO_Handle_t gpioButton;
    memset(&gpioButton, 0, sizeof(gpioButton));

    // button
    gpioButton.pGPIOx = GPIOA;
    gpioButton.pinConfig.number = 0;
    gpioButton.pinConfig.mode = GPIO_PIN_MODE_IN;
    gpioButton.pinConfig.puPdControl = GPIO_PIN_PD;
    GPIO_Init(&gpioButton);
}

int main() {
    RCC->CFGR &= ~(0x3);    // HSI

    setupGPIOForButton();

    memset(&rtcHandle, 0, sizeof rtcHandle);
    rtcHandle.config.pinScl = PIN_SCL;
    rtcHandle.config.pinSda = PIN_SDA;
    rtcHandle.pGPIOx = GPIOB;
    rtcHandle.i2c.pI2Cx = I2C1;
    RTC_Init(&rtcHandle);

//    RTC_Ctrl(&rtcHandle, DISABLE);
//    return 0;

    RTC_DateTime_t dateTime;
    if (!RTC_is_enabled(&rtcHandle)) {
        RTC_Ctrl(&rtcHandle, ENABLE);

        memset(&dateTime, 0, sizeof dateTime);
        dateTime.seconds = 0;
        dateTime.mins = 0;
        dateTime.hours = 17;
        dateTime.format = FORMAT_24H;
        dateTime.dayOfWeek = 7;
        dateTime.day = 3;
        dateTime.month = 9;
        dateTime.year = 23;
        RTC_SetDateTime(&rtcHandle, &dateTime);
    }

    while (1) {
        while (!GPIO_ReadFromInputPin(GPIOA, 0));    // wait for button press
        delay(200);

        memset(&dateTime, 0, sizeof dateTime);
        RTC_GetDateTime(&rtcHandle, &dateTime);
    }
    return 0;
}

