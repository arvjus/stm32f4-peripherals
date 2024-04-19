
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

uint8_t msg[] = "hello arduino!";
uint8_t buff[200];

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
	SPI_Handle_t spi2;
	memset(&spi2, 0, sizeof(spi2));

	spi2.pSPIx = SPI2;
	spi2.config.deviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.config.sclkSpeed = SPI_SCLK_SPEED_DIV256;	// 0.5 MHz
	spi2.config.dff = SPI_DFF_8BIT;					// default
	spi2.config.cpol = SPI_CPO_0_IDLE;				// default
	spi2.config.cpha = SPI_CPHA_1ST_CLK_TRANS;		// default
	spi2.config.ssm = SPI_SSM_DI;					// default
	SPI_Init(&spi2);
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

		// LED ON
		data_out = CMD_LED_CTRL;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_LED_CTRL cmd: %d\n", data_in);
		data_out = 1;	// ON
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_LED_CTRL arg: %d\n", data_in);

		// Read sensor
		data_out = CMD_SENSOR_READ;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_LED_CTRL cmd: %d\n", data_in);
		data_out = 0;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_LED_CTRL value: %d\n", data_in);

		// LED OFF
		data_out = CMD_LED_CTRL;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_LED_CTRL cmd: %d\n", data_in);
		data_out = 0;	// OFF
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_LED_CTRL arg: %d\n", data_in);

		// READ ID
		data_out = CMD_ID_READ;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_ID_READ cmd: %d\n", data_in);
		data_out = 0;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_ID_READ len: %d\n", data_in);
		printf("CMD_ID_READ id: ");
#if 1
		memset(buff, 0, sizeof(buff));
		SPI_TransferData(SPI2, buff, buff, data_in);
		printf("%s\n", buff);
#else
		for (int i = 0; i < data_in; i ++) {
			SPI_SendData(SPI2, &data_out, 1);
			SPI_ReceiveData(SPI2, &data_in, 1);
			printf("%c", data_in);
		}
		printf("\n", data_in);
#endif

		// print msg
		data_out = CMD_MSG_WRITE;
		SPI_SendData(SPI2, &data_out, 1);
		SPI_ReceiveData(SPI2, &data_in, 1);
		printf("CMD_MSG_WRITE cmd: %d\n", data_in);
		SPI_TransferData(SPI2, msg, buff, sizeof(msg));

		while(SPI_GetFlag(SPI2, SPI_SR_BSY_FLAG));	// wait 'till all data is sent

		SPI_PeriphCtrl(SPI2, DISABLE);
	}

	return 0;
}
