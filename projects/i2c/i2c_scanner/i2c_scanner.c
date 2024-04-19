
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_usart.h"

extern void stdio_usart_init(GPIO_RegDef_t *pGPIOx, uint8_t pinTx, uint8_t pinRx, USART_RegDef_t *pUSARTx);

#define	PIN_SCL		6
#define	PIN_SDA		7

#define	SLAVE_ADDR	0x27

I2C_Handle_t i2c1Handle;

void setupGPIOsForI2C1()
{
    GPIO_Handle_t i2c1Pins;
    memset(&i2c1Pins, 0, sizeof(i2c1Pins));

    i2c1Pins.pGPIOx = GPIOB;
    i2c1Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
    i2c1Pins.pinConfig.altFunMode = 4;
    i2c1Pins.pinConfig.opType = GPIO_PIN_OPTYPE_OD;
    i2c1Pins.pinConfig.puPdControl = GPIO_PIN_PU;
    i2c1Pins.pinConfig.speed = GPIO_PIN_SPEED_MEDIUM;

    // SCL
    i2c1Pins.pinConfig.number = PIN_SCL;
    GPIO_Init(&i2c1Pins);

    // SDA
    i2c1Pins.pinConfig.number = PIN_SDA;
    GPIO_Init(&i2c1Pins);
}

void setupI2C1()
{
    memset(&i2c1Handle, 0, sizeof(i2c1Handle));

    i2c1Handle.pI2Cx = I2C1;
    i2c1Handle.config.ackControl = I2C_ACK_EN;
    i2c1Handle.config.fmDutyCycle = I2C_FM_DUTY_2;
    i2c1Handle.config.sclSpeed = I2C_SCL_SPEED_SM;
    I2C_Init(&i2c1Handle);
}

int main()
{
    RCC_SysClkSwitch(RCC_SYSCLK_HSE);
    stdio_usart_init(GPIOA, 2, 3, USART2);

    setupGPIOsForI2C1();
    setupI2C1();
    I2C_PeriphCtrl(I2C1, ENABLE);

    uint8_t addr = 0, found;
    while (++ addr <= 127) {
        found = I2C_MasterProbe(&i2c1Handle, addr);
        if (found) {
            printf("Found I2C addr=0x%x (%d)\r\n", addr, addr);
            break;
        }
    }
    if (addr >= 127)
        printf("Not found any I2C slaves\r\n");

    I2C_PeriphCtrl(I2C1, DISABLE);
    return 0;
}
