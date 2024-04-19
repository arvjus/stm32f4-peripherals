
#include <string.h>
#include <stdio.h>
#include "stm32f407xx_usart.h"
#include "stm32f407xx_gpio.h"

#define	PIN_TX		2
#define	PIN_RX		3


USART_Handle_t usartHandle;
uint8_t msg[] = "uart testing..\r\n";

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

void setupGPIOsForUSART2()
{
	GPIO_Handle_t usart2Pins;
	memset(&usart2Pins, 0, sizeof(usart2Pins));

	usart2Pins.pGPIOx = GPIOA;
	usart2Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
	usart2Pins.pinConfig.altFunMode = 7;
	usart2Pins.pinConfig.opType = GPIO_PIN_OPTYPE_PP;
	usart2Pins.pinConfig.puPdControl = GPIO_PIN_PU;
	usart2Pins.pinConfig.speed = GPIO_PIN_SPEED_MEDIUM;

	// TX
	usart2Pins.pinConfig.number = PIN_TX;
	GPIO_Init(&usart2Pins);

	// RX
	usart2Pins.pinConfig.number = PIN_RX;
	GPIO_Init(&usart2Pins);
}

void setupUSART2()
{
	memset(&usartHandle, 0, sizeof(usartHandle));

	usartHandle.pUSARTx = USART2;
	usartHandle.config.mode = USART_MODE_ONLY_TX;
	usartHandle.config.baudRate = USART_STD_BAUD_115200;
	usartHandle.config.stopBits = USART_PARITY_DI;
	usartHandle.config.wordLength = USART_WORDLEN_8BITS;
	usartHandle.config.stopBits = USART_STOPBITS_1;
	usartHandle.config.hwFlowCtrl = USART_HW_FLOW_CTRL_NONE;
	USART_Init(&usartHandle);
}

int main()
{
	initialise_monitor_handles();
	printf("application is running\n");

	setupGPIOForButton();

	setupGPIOsForUSART2();

	setupUSART2();

	USART_PeriphCtrl(USART2, ENABLE);

	while(1) {
		while(!GPIO_ReadFromInputPin(GPIOA, 0));	// wait for button press
		delay(200);

		USART_SendData(&usartHandle, msg, strlen((char*)msg));
	}

	return 0;
}

