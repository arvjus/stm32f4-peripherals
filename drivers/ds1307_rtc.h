
#ifndef STM32F4_PERIPHERALS_DS1307_RTC_H
#define STM32F4_PERIPHERALS_DS1307_RTC_H

#include <stdbool.h>
#include "stm32f407xx_i2c.h"

typedef struct {
    uint8_t pinScl;			/*!< possible values 0-15 for given GPIOx >*/
    uint8_t pinSda;			/*!< possible values 0-15 for given GPIOx >*/
} RTC_Config_t;

typedef struct {
    RTC_Config_t    config;
    GPIO_RegDef_t   *pGPIOx;
    I2C_Handle_t    i2c;
} RTC_Handle_t;

typedef struct
{
    uint8_t seconds;
    uint8_t mins;
    uint8_t hours;
    uint8_t format;
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t dayOfWeek;
} RTC_DateTime_t;

/*
 * @RTC_Format
 */
#define FORMAT_AM 0
#define FORMAT_PM 1
#define FORMAT_24H 2

void RTC_Init(RTC_Handle_t *pRTCHandle);
void RTC_Ctrl(RTC_Handle_t *pRTCHandle, uint8_t on_off);
bool RTC_is_enabled(RTC_Handle_t *pRTCHandle);
void RTC_SetDateTime(RTC_Handle_t *pRTCHandle, RTC_DateTime_t *pRTC_DateTime);
void RTC_GetDateTime(RTC_Handle_t *pRTCHandle, RTC_DateTime_t *pRTC_DateTime);

#endif //STM32F4_PERIPHERALS_DS1307_RTC_H
