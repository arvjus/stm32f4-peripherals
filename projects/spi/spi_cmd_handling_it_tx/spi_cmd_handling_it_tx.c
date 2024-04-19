
#include <string.h>
#include <stdio.h>
#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"

/*
 * SPI2
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCLK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * Alt function mode 5
 * (Alternate function mapping - datasheet p63)
 */

#define	PIN_NSS		12
#define	PIN_SCLK	13
#define	PIN_MISO	14
#define	PIN_MOSI	15

#define CMD_LED_CTRL 0x50
#define CMD_SENSOR_READ 0x51
#define CMD_ID_READ 0x52
#define CMD_MSG_WRITE 0x53

SPI_Handle_t spi2Handle;

char msg[] = "hello arduino!";
char buff[200];
uint8_t rx_len = 0, rx_completed = 0;

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

void setupGPIOsForSPI2()
{
	GPIO_Handle_t spi2Pins;
	memset(&spi2Pins, 0, sizeof(spi2Pins));

	spi2Pins.pGPIOx = GPIOB;
	spi2Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
	spi2Pins.pinConfig.altFunMode = 5;
	spi2Pins.pinConfig.opType = GPIO_PIN_OPTYPE_PP; // default
	spi2Pins.pinConfig.puPdControl = GPIO_PIN_NONE_PU_PD; // default

	// NSS
	spi2Pins.pinConfig.number = PIN_NSS;
	GPIO_Init(&spi2Pins);

	// SCLK
	spi2Pins.pinConfig.number = PIN_SCLK;
	GPIO_Init(&spi2Pins);

	// MISO
	spi2Pins.pinConfig.number = PIN_MISO;
	GPIO_Init(&spi2Pins);

	// MOSI
	spi2Pins.pinConfig.number = PIN_MOSI;
	GPIO_Init(&spi2Pins);
}

void setupSPI2()
{
	memset(&spi2Handle, 0, sizeof(spi2Handle));

	spi2Handle.pSPIx = SPI2;
	spi2Handle.config.busConfig = SPI_BUS_CONFIG_FD; 		// default
	spi2Handle.config.deviceMode = SPI_DEVICE_MODE_MASTER;
	spi2Handle.config.sclkSpeed = SPI_SCLK_SPEED_DIV32;	// 2 MHz
	spi2Handle.config.dff = SPI_DFF_8BIT;					// default
	spi2Handle.config.cpol = SPI_CPO_0_IDLE;				// default
	spi2Handle.config.cpha = SPI_CPHA_1ST_CLK_TRANS;		// default
	spi2Handle.config.ssm = SPI_SSM_DI;					// default
	SPI_Init(&spi2Handle);
}

int main()
{
	uint8_t data_out, data_in;

 	initialise_monitor_handles();
	printf("application is running\n");


	setupGPIOForButton();

	setupGPIOsForSPI2();

	setupSPI2();

	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {
		while(!GPIO_ReadFromInputPin(GPIOA, 0));	// wait for button press
		delay(200);

		SPI_PeriphCtrl(SPI2, ENABLE);

		// print msg
		data_out = CMD_MSG_WRITE;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_MSG_WRITE cmd: %d\n", data_in);

		IRQEnable(IRQ_NO_SPI2, ENABLE);
		SPI_SendDataIT(&spi2Handle, (uint8_t *)msg, sizeof(msg));
		SPI_ReceiveDataIT(&spi2Handle, (uint8_t *)buff, sizeof(msg));
		IRQEnable(IRQ_NO_SPI2, DISABLE);


		while(SPI_GetFlag(SPI2, SPI_SR_BSY_FLAG));	// wait 'till all data is sent

		SPI_PeriphCtrl(SPI2, DISABLE);
	}

	return 0;
}

void SPI2_IRQHandler()
{
	SPI_IRQHandling(&spi2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	if (event == SPI_EVENT_TX_CMPLT) {
		printf("CMD_MSG_WRITE completed\n");
	}
}

