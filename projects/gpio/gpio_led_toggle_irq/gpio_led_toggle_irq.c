
#include <stm32f407xx.h>
#include <stm32f407xx_gpio.h>
#include <string.h>

uint32_t volatile *pEXTTIPendReg			= (uint32_t*) (0x40013C00 + 0x14);
uint32_t volatile *pClkCtrlReg				= (uint32_t*) (0x40023800 + 0x30);
uint32_t volatile *pClkCtrlRegApb2			= (uint32_t*) (0x40023800 + 0x44);
uint32_t volatile *pGPIOAModeReg 			= (uint32_t*) (0x40020000 + 0x00);
uint32_t volatile *pEXTIMaskReg 			= (uint32_t*) (0x40013C00 + 0x00);
uint32_t volatile *pEXTTIEdgeCtrlReg		= (uint32_t*) (0x40013C00 + 0x08);
uint32_t volatile *pNVICIRQEnReg 			= (uint32_t*) 0xE000E100;


int main(void)
{
	// toggle led on PD12 whenever interrupt is triggered by pressing a button on PA0 (falling edge)

	GPIO_Handle_t gpioButton, gpioLed;
	memset(&gpioButton, 0, sizeof(gpioButton));
	memset(&gpioLed, 0, sizeof(gpioLed));

	// button
	gpioButton.pGPIOx = GPIOA;
	gpioButton.pinConfig.number = GPIO_PIN_NO_0;
	gpioButton.pinConfig.mode = GPIO_PIN_MODE_IT_RT;
	gpioButton.pinConfig.opType = GPIO_PIN_OPTYPE_OD;
	gpioButton.pinConfig.puPdControl = GPIO_PIN_PD;
	GPIO_Init(&gpioButton);
	IRQEnable(IRQ_NO_EXTI0, ENABLE);
	IRQPriority(IRQ_NO_EXTI0, NVIC_IRQ_PRIO_15);


	// led
	gpioLed.pGPIOx = GPIOD;
	gpioLed.pinConfig.number = GPIO_PIN_NO_12;
	gpioLed.pinConfig.mode = GPIO_PIN_MODE_OUT;
	gpioLed.pinConfig.opType = GPIO_PIN_OPTYPE_PP;
	gpioLed.pinConfig.puPdControl = GPIO_PIN_NONE_PU_PD;
	GPIO_Init(&gpioLed);
}

void EXTI0_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

void HardFault_Handler()
{
}
