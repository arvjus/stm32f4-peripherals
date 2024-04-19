
#include <string.h>
#include <stdio.h>
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_gpio.h"

#define	PIN_SCL		6
#define	PIN_SDA		9

#define	OWN_ADDR	0x69

#define	CMD_GET_LEN	0x51
#define	CMD_GET_MSG	0x52


I2C_Handle_t i2c1Handle;
uint8_t msg[] = "sm32f407 slave is sending";

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
	i2c1Handle.config.ackControl = I2C_ACK_EN;
	i2c1Handle.config.deviceAddress = OWN_ADDR;
	i2c1Handle.config.fmDutyCycle = I2C_FM_DUTY_2;
	i2c1Handle.config.sclSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&i2c1Handle);
}

int main()
{
	setupGPIOForButton();

	setupGPIOsForI2C1();

	setupI2C1();

	IRQEnable(IRQ_NO_I2C1_EV, ENABLE);
	IRQEnable(IRQ_NO_I2C1_ER, ENABLE);


	I2C_EventCtrl(I2C1, ENABLE);

	I2C_PeriphCtrl(I2C1, ENABLE);

	I2C_AckCtrl(I2C1, ENABLE);

	while(1);

	return 0;
}

void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&i2c1Handle);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&i2c1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	static uint8_t cmd = 0xff, cnt = 0;

	if (event == I2C_EVENT_DATA_REQ) {
		if (cmd == CMD_GET_LEN) {
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)msg));
		} else if (cmd == CMD_GET_MSG) {
			I2C_SlaveSendData(pI2CHandle->pI2Cx, msg[cnt ++]);
		}
	} else if (event == I2C_EVENT_DATA_RCV) {
		cmd = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	} else if (event == I2C_EVENT_STOP) {
		// slave RX
	} else if (event == I2C_ERROR_AF) {
		// slave TX
		// master does not need more data
		cmd = 0xff;
		cnt = 0;
	}
}
