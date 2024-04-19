
#include <string.h>
#include <stdio.h>
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_gpio.h"

#define	PIN_SCL		6
#define	PIN_SDA		9

#define	OWN_ADDR	0x61
#define	SLAVE_ADDR	0x68

I2C_Handle_t i2c1Handle;
uint8_t some_data[] = "We're testing I2C master TX\n";	// max 32 bytes

extern void initialise_monitor_handles();

void setupGPIOForButton()
{
	GPIO_Handle_t gpioButton;
	memset(&gpioButton, 0, sizeof(gpioButton));

	// button
	gpioButton.pGPIOx = GPIOA;
	gpioButton.pinConfig.number = 0;
	gpioButton.pinConfig.mode = GPIO_PIN_MODE_IN;
	gpioButton.pinConfig.puPdControl = GPIO_PIN_PD;
	GPIO_Init(&gpioButton);
}

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
	i2c1Handle.config.ackControl =I2C_ACK_EN;
	i2c1Handle.config.deviceAddress = OWN_ADDR;
	i2c1Handle.config.fmDutyCycle = I2C_FM_DUTY_2;
	i2c1Handle.config.sclSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&i2c1Handle);
}

int main()
{
	initialise_monitor_handles();
	printf("application is running\n");

	setupGPIOForButton();

	setupGPIOsForI2C1();

	setupI2C1();

	while (1) {
		while(!GPIO_ReadFromInputPin(GPIOA, 0));	// wait for button press
		delay(200);

		I2C_PeriphCtrl(I2C1, ENABLE);
		I2C_MasterSendData(&i2c1Handle, some_data, strlen((const char *)some_data), SLAVE_ADDR, I2C_SR_DI);
		I2C_PeriphCtrl(I2C1, DISABLE);
	}
	return 0;
}

