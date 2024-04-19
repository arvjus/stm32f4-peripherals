
#include <string.h>
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

#define	PIN_TRIG	11
#define	PIN_NSS		12
#define	PIN_SCLK	13
#define	PIN_MISO	14
#define	PIN_MOSI	15


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

	// TRIG
	spi2Pins.pinConfig.number = PIN_TRIG;
	spi2Pins.pinConfig.mode = GPIO_PIN_MODE_OUT;
	GPIO_Init(&spi2Pins);

}

void setupSPI2()
{
	SPI_Handle_t spi2;
	memset(&spi2, 0, sizeof(spi2));

	spi2.pSPIx = SPI2;
	spi2.config.deviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.config.sclkSpeed = SPI_SCLK_SPEED_DIV2;	// default
	spi2.config.dff = SPI_DFF_8BIT;					// default
	spi2.config.cpol = SPI_CPO_0_IDLE;				// default
	spi2.config.cpha = SPI_CPHA_1ST_CLK_TRANS;		// default
	spi2.config.ssm = SPI_SSM_EN;
	SPI_Init(&spi2);
}

int main()
{
	//uint16_t msg[] = { 0x0001, 0xff00, 0xf101 };
	char msg[] = "Hello World!";

	//GPIOA_CLK_EN();
	setupGPIOsForSPI2();

	//SPI2_CLK_EN();
	setupSPI2();

	SPI_PeriphCtrl(SPI2, ENABLE);

	GPIO_WriteToOutputPin(GPIOB, PIN_TRIG, HIGH);

	while(1) {
		GPIO_WriteToOutputPin(GPIOB, PIN_TRIG, LOW);

		SPI_SendData(SPI2, (uint8_t*) msg, sizeof(msg));

		GPIO_WriteToOutputPin(GPIOB, PIN_TRIG, HIGH);

		delay(100);
	}

	SPI_PeriphCtrl(SPI2, DISABLE);


	return 0;
}
