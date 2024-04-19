
#include "stm32f407xx_gpio.h"

int main(void)
{
	// toggle led on PD12 whenever button on PA0 is pressed

	GPIO_Handle_t gpioButton, gpioLed;

	// button
	gpioButton.pGPIOx = GPIOA;
	gpioButton.pinConfig.number = 0;
	gpioButton.pinConfig.mode = GPIO_PIN_MODE_IN;
	gpioButton.pinConfig.puPdControl = GPIO_PIN_PD;
	GPIO_Init(&gpioButton);

	// led
	gpioLed.pGPIOx = GPIOD;
	gpioLed.pinConfig.number = 12;
	gpioLed.pinConfig.mode = GPIO_PIN_MODE_OUT;
	gpioLed.pinConfig.opType = GPIO_PIN_OPTYPE_PP;
	gpioLed.pinConfig.puPdControl = GPIO_PIN_NONE_PU_PD;
	GPIO_Init(&gpioLed);

	while (1) {
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)) {
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			delay(1000);
		}
	}
}
