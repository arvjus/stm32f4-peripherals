
#include <string.h>
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_i2c.h"
#include "ds1307_rtc.h"

#define    SLAVE_ADDR    0x68

static uint8_t bcdToByte(uint8_t value) {
    uint8_t tmp = 0;
    tmp = ((uint8_t) (value & (uint8_t) 0xF0) >> (uint8_t) 0x4) * 10;
    return (tmp + (value & (uint8_t) 0x0F));
}

static uint8_t byteToBcd(uint8_t value) {
    uint8_t bcdhigh = 0;
    while (value >= 10) {
        bcdhigh++;
        value -= 10;
    }
    return ((uint8_t) (bcdhigh << 4) | value);
}

static uint8_t read_from_address(I2C_Handle_t *pI2CHandle, uint8_t addr)
{
    uint8_t value;
    I2C_MasterSendData(pI2CHandle,&addr,1,SLAVE_ADDR,I2C_SR_DI);
    I2C_MasterReceiveData(pI2CHandle,&value,1,SLAVE_ADDR,I2C_SR_EN);
    return value;
}

static void write_to_address(I2C_Handle_t *pI2CHandle, uint8_t addr, uint8_t value)
{
    uint8_t data[] = { addr, value };
    I2C_MasterSendData(pI2CHandle,data,2,SLAVE_ADDR,I2C_SR_DI);
}

void RTC_Init(RTC_Handle_t *pRTCHandle) {
    // GPIO
    GPIO_Handle_t i2c1Pins;
    memset(&i2c1Pins, 0, sizeof(i2c1Pins));

    i2c1Pins.pGPIOx = pRTCHandle->pGPIOx;
    i2c1Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
    i2c1Pins.pinConfig.altFunMode = 4;
    i2c1Pins.pinConfig.opType = GPIO_PIN_OPTYPE_OD;
    i2c1Pins.pinConfig.puPdControl = GPIO_PIN_PU;
    i2c1Pins.pinConfig.speed = GPIO_PIN_SPEED_MEDIUM;
    // SCL
    i2c1Pins.pinConfig.number = pRTCHandle->config.pinScl;
    GPIO_Init(&i2c1Pins);
    // SDA
    i2c1Pins.pinConfig.number = pRTCHandle->config.pinSda;
    GPIO_Init(&i2c1Pins);

    // I2C
    pRTCHandle->i2c.config.ackControl = I2C_ACK_EN;
    pRTCHandle->i2c.config.fmDutyCycle = I2C_FM_DUTY_2;
    pRTCHandle->i2c.config.sclSpeed = I2C_SCL_SPEED_SM;
    I2C_Init(&pRTCHandle->i2c);

    I2C_PeriphCtrl(pRTCHandle->i2c.pI2Cx, ENABLE);
}

void RTC_Ctrl(RTC_Handle_t *pRTCHandle, uint8_t on_off)
{
    uint8_t data[] = { 0, on_off ? 0 : (1 << 7) };
    I2C_MasterSendData(&pRTCHandle->i2c, data, 2, SLAVE_ADDR, I2C_SR_DI);
}

bool RTC_is_enabled(RTC_Handle_t *pRTCHandle)
{
    uint8_t addr = 0x00;
    I2C_MasterSendData(&pRTCHandle->i2c, &addr, 1, SLAVE_ADDR, I2C_SR_DI);
    I2C_MasterReceiveData(&pRTCHandle->i2c, &addr, 1, SLAVE_ADDR, I2C_SR_EN);
    return !(addr >> 7);
}

void RTC_SetDateTime(RTC_Handle_t *pRTCHandle, RTC_DateTime_t *pRTC_DateTime)
{
    uint8_t seconds = byteToBcd(pRTC_DateTime->seconds);
    seconds &= ~(1 << 7);
    write_to_address(&pRTCHandle->i2c, 0x00, seconds);
    write_to_address(&pRTCHandle->i2c, 0x01, byteToBcd(pRTC_DateTime->mins));

    if (pRTC_DateTime->format == FORMAT_24H) {
        pRTC_DateTime->hours &= ~(1 << 6);
    } else {
        pRTC_DateTime->hours |= (1 << 6);
        if (pRTC_DateTime->format == FORMAT_PM) {
            pRTC_DateTime->hours |= (1 << 5);
        } else {
            pRTC_DateTime->hours &= ~(1 << 5);
        }
    }
    write_to_address(&pRTCHandle->i2c, 0x02, byteToBcd(pRTC_DateTime->hours));
    write_to_address(&pRTCHandle->i2c, 0x03, byteToBcd(pRTC_DateTime->dayOfWeek));
    write_to_address(&pRTCHandle->i2c, 0x04, byteToBcd(pRTC_DateTime->day));
    write_to_address(&pRTCHandle->i2c, 0x05, byteToBcd(pRTC_DateTime->month));
    write_to_address(&pRTCHandle->i2c, 0x06, byteToBcd(pRTC_DateTime->year));
}

void RTC_GetDateTime(RTC_Handle_t *pRTCHandle, RTC_DateTime_t *pRTC_DateTime)
{
    uint8_t seconds = read_from_address(&pRTCHandle->i2c, 0x00);
    seconds &= ~(1 << 7);
    pRTC_DateTime->seconds = bcdToByte(seconds);
    pRTC_DateTime->mins = bcdToByte(read_from_address(&pRTCHandle->i2c, 0x01));
    pRTC_DateTime->hours = read_from_address(&pRTCHandle->i2c, 0x02);
    if (pRTC_DateTime->hours & (1 << 6)) {
        //12h format
        if (pRTC_DateTime->hours & (1 << 5)) {
            pRTC_DateTime->format = FORMAT_PM;
        } else {
            pRTC_DateTime->format = FORMAT_AM;
        }
        pRTC_DateTime->hours &= ~(0X3 << 5);
    } else {
        pRTC_DateTime->format = FORMAT_24H;
    }
    pRTC_DateTime->hours = bcdToByte(pRTC_DateTime->hours);
    pRTC_DateTime->dayOfWeek = bcdToByte(read_from_address(&pRTCHandle->i2c, 0x03));
    pRTC_DateTime->day = bcdToByte(read_from_address(&pRTCHandle->i2c, 0x04));
    pRTC_DateTime->month = bcdToByte(read_from_address(&pRTCHandle->i2c, 0x05));
    pRTC_DateTime->year = bcdToByte(read_from_address(&pRTCHandle->i2c, 0x06));
}

