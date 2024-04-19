#include <stm32f407xx_gpio.h>

uint8_t gpioToPos(GPIO_RegDef_t *pGPIOx);

void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t en_di)
{
	uint8_t pin = gpioToPos(pGPIOx);
	if (en_di == ENABLE)
		GPIOX_CLK_EN_BIT(pin);
	else
		GPIOX_CLK_DI_BIT(pin);
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t pinX2Pos = pGPIOHandle->pinConfig.number * 2;

	// enable clock
	GPIO_ClkCtrl(pGPIOHandle->pGPIOx, ENABLE);

	// mode
	if (pGPIOHandle->pinConfig.mode <= GPIO_PIN_MODE_ANALOG) {
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pinX2Pos);
		pGPIOHandle->pGPIOx->MODER |= pGPIOHandle->pinConfig.mode << pinX2Pos;
	} else {
		// interrupt mode

		// port selection in SYSCFG_EXTICR
		uint8_t tmp1 = pGPIOHandle->pinConfig.number / 4;
		uint8_t tmp2 = pGPIOHandle->pinConfig.number % 4;
		uint8_t portcode = gpioToPos(pGPIOHandle->pGPIOx);
		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[tmp1] = (portcode << (tmp2 * 4));

		// enable exti interrupt delivery
		EXTI->IMR |= (1 << pGPIOHandle->pinConfig.number);

		// configure FTSR, RTSR
		if (pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_IT_FT || pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_IT_RFT)
			EXTI->FTSR |= (1 << pGPIOHandle->pinConfig.number);
		if (pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_IT_FT)
			EXTI->RTSR &= ~(1 << pGPIOHandle->pinConfig.number);
		if (pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_IT_RT || pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_IT_RFT)
			EXTI->RTSR |= (1 << pGPIOHandle->pinConfig.number);
		if (pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_IT_RT)
			EXTI->FTSR &= ~(1 << pGPIOHandle->pinConfig.number);
	}

	// speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pinX2Pos);
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->pinConfig.speed << pinX2Pos);

	// pu/pd settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pinX2Pos);
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->pinConfig.puPdControl << pinX2Pos);

	// optype
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->pinConfig.number);
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->pinConfig.opType << pGPIOHandle->pinConfig.number);

	// alt funcionality
	if (pGPIOHandle->pinConfig.mode == GPIO_PIN_MODE_ALTFN) {
		uint8_t tmp1, tmp2;

		tmp1 = pGPIOHandle->pinConfig.number / 8;
		tmp2 = pGPIOHandle->pinConfig.number % 8;
		pGPIOHandle->pGPIOx->AFR[tmp1] &= ~(0xF << (tmp2 * 4));
		pGPIOHandle->pGPIOx->AFR[tmp1] |= (pGPIOHandle->pinConfig.altFunMode << (tmp2 * 4));
	}
}

void GPIO_Reset(GPIO_RegDef_t *pGPIOx)
{
	uint8_t pin = gpioToPos(pGPIOx);
	GPIOX_REG_RESET_BIT(pin);
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin)
{
	return (uint8_t)((pGPIOx->IDR >> pin) & 0x00000001);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value)
{
	if (value == SET)
		pGPIOx->ODR |= (1 << pin);
	else
		pGPIOx->ODR &= ~(1 << pin);
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin)
{
	pGPIOx->ODR ^= (1 << pin);
}

void GPIO_IRQHandling(uint8_t pin)
{
	// clear EXTI PR
	if (EXTI->PR & (1 << pin))
		EXTI->PR |= (1 << pin);

}

uint8_t gpioToPos(GPIO_RegDef_t *pGPIOx)
{
	const static GPIO_RegDef_t *gpios[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI };
	for (int i = 0; i < sizeof(gpios) / sizeof(gpios[0]); i++) {
		if (pGPIOx == gpios[i])
			return i;
	}
	assert(0);
}
